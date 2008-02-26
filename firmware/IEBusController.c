/*--------------------------------------------------------------------------------------------------

  Name         :  IEBusController.c

  Description  :  This program will read packets off of the IEBus and transmit them to a pc over a 
                  serial connection. The format of the data sent to the pc is:
				    
					 "~012AA801062E4FFF120C8B"

  MCU          :  ATmega8 @ 8 MHz using internal RC & watchdog.
  
  Author       :  2007-08-25 - Robert Newton (rob@angrycamel.com)
                  Based on code by Louis Frigon (info@sigmaobjects.com)

  Copyright    :  (c) 2007 Robert Newton (www.angrycamel.com)

  History      :  2007-08-25 - v0.1 Initial testing for proof of concept base on firmware by
                                    Louis Frigon at www.sigmaobjects.com

--------------------------------------------------------------------------------------------------*/
#include <avr/interrupt.h>
#include <avr/wdt.h> 
#include <avr/io.h>
#include <stdio.h>

#include "Globals.h"
#include "USART.h"
#include "IEBus.h"
#include "string.h"

#define FIRMWARE_VERSION    "v0.8.3"
#define FIRMWARE_DATE       __DATE__

typedef enum
{
    CMD_ENABLE_FILTERS,
	CMD_DISABLE_FILTERS

} commands;

typedef struct 
{
	byte				commandID;			// Command Identifier
	word				parm1;				// Parameter
	word				parm2;				// Parameter
} commandStruct;

typedef         commandStruct		rxCommand;

/*--------------------------------------------------------------------------------------------------
                                         Prototypes
--------------------------------------------------------------------------------------------------*/
void			InitMCU					( void );
void			InitTimers				( void );
bool			getRxCommand			( IEBus_Message* txMsg );
void			sendMessageOverSerial	( IEBus_Message *msg );
void			sendErrorOverSerial		( IEBus_Error *err );

/*--------------------------------------------------------------------------------------------------
                                      Global Variables
--------------------------------------------------------------------------------------------------*/
bool			processingCommand;
bool        	g_Emulating;
word			g_EmuMaster;
IEBus_Message trigger;

/*--------------------------------------------------------------------------------------------------

  Name         :  InitMCU

  Description  :  Performs MCU initialization.

  Argument(s)  :  None.

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void InitMCU ( void )
{
    // Init IO pins
    LED_DDR |= LEDOUT | LED2OUT | RELAYOUT | RELAY2OUT;
    activityIndicatorOff();
    videoOverrideOff();
    modeIndicatorOff();
    IEBusPassthroughOff();

    InitTimers();

    USART_init();

    // Preset AVC bus driver output pins but leave pins tri-stated until we need to use them.
    PORTD |= _BV(PD3);   // PD3 (+) high.
    PORTD &= ~_BV(PD2);  // PD2 (-) low.
    
    // Enable watchdog @ ~2 sec.
    wdt_enable( WDTO_2S );

    // Enable system interrupts
    sei();
}

/*--------------------------------------------------------------------------------------------------

  Name         :  InitTimers

  Description  :  Timers initialization routine.

  Argument(s)  :  None.

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void InitTimers ( void )
{
    // Timer 0 prescaler = 8 ( 1 count / us )
    TCCR0 = _BV(CS01);

    //  Timer 1 in CTC mode, prescaler = 256
    TCCR1B = _BV(WGM12) | _BV(CS12);

    //  Compare value for 1 sec tick @ 8 MHz.
    OCR1A = 0x7A12;

    //  Timer 1A compare match interrupt enable.
    TIMSK = _BV(OCIE1A);
}

/*--------------------------------------------------------------------------------------------------

  Name         :  I/O Pin Functions

--------------------------------------------------------------------------------------------------*/
inline void activityIndicatorOn ( void )
{
    LED_PORT |= LEDOUT;
}

inline void activityIndicatorOff ( void )
{
    LED_PORT &= ~LEDOUT;
}

inline void modeIndicatorOn ( void )
{
    LED_PORT |= LED2OUT;
}

inline void modeIndicatorOff ( void )
{
    LED_PORT &= ~LED2OUT;
}

inline void videoOverrideOn ( void )
{
    LED_PORT |= RELAYOUT;
}

inline void videoOverrideOff ( void )
{
    LED_PORT &= ~RELAYOUT;
}

inline void IEBusPassthroughOn ( void )
{
    LED_PORT |= RELAY2OUT;
}

inline void IEBusPassthroughOff ( void )
{
    LED_PORT &= ~RELAY2OUT;
}

/*--------------------------------------------------------------------------------------------------

  Name         :  convertToHex

  Description  :  Return a string as a hex formatted byte array.

  Argument(s)  :  str -> pointer to constant string to send (strings in Flash).

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void convertToHex( const char *str )
{
    char c;
	char i = 0;
	char buffer[16];
    
    while ( (c = pgm_read_byte_near( str++ )) )
    {
        itoa (c,buffer,16);
		i++;
    }
}

/*--------------------------------------------------------------------------------------------------

  Name         :  delay_ms

  Description  :  General short delays

  Argument(s)  :  uint16_t x (milliseconds to delay for)

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 80 ; y++){
      for ( z = 0 ; z < 40 ; z++){
        asm volatile ("nop");
      }
    }
  }
}

/*--------------------------------------------------------------------------------------------------

  Name         :  getRxCommand

  Description  :  Recieve a command from the pc over the serial port. Exit with false if the first
                  character in the rxBuffer is not the start byte for a command (!).

				  Command format:

				     Start [1 byte]
					 | CommandID [1 byte]
					 | | Parm1 [2 bytes]  
					 | | |   Parm 2 [2 bytes]
					 | | |   |   |End [1 bytes]
					[~,C,1,1,2,2,^]

  Argument(s)  :  None.

  Return value :  (bool)-> True if successful.

--------------------------------------------------------------------------------------------------*/
bool getRxCommand(IEBus_Message* txMsg)
{
	byte lastChar = (USART_rxByteCount - 1);
	bool debug = FALSE;

	//Is there a complete command found in the buffer?
    if ( USART_rxByteCount > 0 )
	{
		if(debug)
		{
			USART_putCString( PSTR("\r\nProcessing RXBuffer: ") );
			for (short i=0;i<USART_rxByteCount;i++) {
				sprintf(USART_txBuffer, "%X", USART_rxBuffer[i]);
				USART_putString(USART_txBuffer);
				if(i!=(USART_rxByteCount-1)) {
					USART_putCString( PSTR(":") );
				}
			}
		}
        if ( USART_rxBuffer[0] == 0x7E )						 // '~' indicates the start of a command
        {
			if(debug)
			{
				USART_putCString( PSTR("\r\n\tFound Command Start: ") );
				USART_putString( USART_rxBuffer );
			}
            if ( USART_rxBuffer[lastChar] == 0x5E )				// '^' indicates the end of a command
			{   
				if(!debug)
				{
					USART_putCString( PSTR("\r\n\tFound Command End: ") );
					USART_putString( USART_rxBuffer );
				}
				memcpy(&txMsg->broadcast,	&USART_rxBuffer[1], 1);
				memcpy(&txMsg->master,		&USART_rxBuffer[2], 2);
				memcpy(&txMsg->slave,		&USART_rxBuffer[4], 2);
				memcpy(&txMsg->control,		&USART_rxBuffer[6], 1);
				memcpy(&txMsg->datasize,	&USART_rxBuffer[7], 1);
				for (short i=8;i<(USART_rxByteCount-1);i++) {
					memcpy(&txMsg->data[i-8],	&USART_rxBuffer[i], 1);
				}

				if(debug)
				{
					sprintf(USART_txBuffer, "\r\ntxMsg->master: 0x%X", txMsg->master);
					USART_putString(USART_txBuffer);
				}
				USART_rxBufferClear();
                return TRUE;
            }
            else
			{
				if(debug)
				{
					sprintf( USART_txBuffer, "\r\n\tLast char: 0x%X <> 0x5E (^)", USART_rxBuffer[lastChar]);
					USART_putString(USART_txBuffer);
				}
                return FALSE;								// command is still transmitting (get it on the next iter)
            }
        }
        else												// else if doesn't start with '!' then corrupt command
		{
			if(debug)
			{
				sprintf(USART_txBuffer, "\r\n\tFirst char: 0x%X <> 0x7E (~)", USART_rxBuffer[0]);
				USART_putString(USART_txBuffer);
			}
			// Ignore anything that is not a complete command
            USART_rxBufferClear();
            return FALSE;
        }
    }
    else
    {
        return FALSE;
    }
}

void sendMessageOverSerial(IEBus_Message *msg)
{
	//Format example: "~012AA801062E4FFF120C8B"
	sprintf(USART_txBuffer, "~%.2X%.4X%.4X%.2X%.2X", msg->broadcast, msg->master, msg->slave, msg->control, msg->datasize);
	USART_putString(USART_txBuffer);
	for(byte i=0; i < msg->datasize; i++){
		sprintf(USART_txBuffer, "%.2X", msg->data[i]);
		USART_putString(USART_txBuffer);
	}
}

void sendErrorOverSerial(IEBus_Error *err)
{
	//Format example: "~*3:Parity error getting Control^\r\n"
	sprintf(USART_txBuffer, "~*%d:%s^", err->code, err->description);
	USART_putString(USART_txBuffer);
}

IEBus_Message getTriggerMessageFromEEPROM()
{
	//Read a message from the EEPROM that will be compared to the current
	//message to determine if it matches so the mode can be switched

	//TODO: Replace this hard coded value with a read from EEPROM
	// once we get the read from EEPROM working
	//TODO: Provide a method for the user to send down the trigger
	// message from IEBus Studio and have it written into the EEPROM

	//For now let the cancel button be the button to control switching
    IEBus_Message cancelButton = {1,0x131,0x183,0xF,7,
        {0x37,0x31,0x0D,0x00,0x01,0x09,0x80,
        0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0}};  //Cancel Button
    
    return cancelButton; 
}

word getEmulatedMasterFromEEPROM()
{
	//Read in the master id to emulate from the EEPROM

	//TODO: Replace this hard coded value with a read from EEPROM
	// once we get the read from EEPROM working
	//TODO: Provide a method for the user to send down the trigger
	// message from IEBus Studio and have it written into the EEPROM

	//For now lets just use the headunit
    return 0x131;  //Headunit
}

void listenForModeSwitch(IEBus_Message *msg)
{
	//This will switch us from passive listening mode to actively ACK'ing
	//messages from a particular master id
	if ( IEBus_cmpMessage(&msg, &trigger) ){
		//Flip the relays
		if ( g_Emulating ){  //Emulating is on, so turn it off
            USART_putCString( PSTR("Relays and LEDs off...\r\n") );
			videoOverrideOff();    //Switches the video source
			IEBusPassthroughOff();   //Switches the IEBus +/-
			modeIndicatorOff();     //Switches the Emulating LED
		}else{
            USART_putCString( PSTR("Relays and LEDs on...\r\n") );
			videoOverrideOn();
			IEBusPassthroughOn();
			modeIndicatorOn();
		}
		g_Emulating = !g_Emulating;
	}
}

int main ( void )
{
    InitMCU();

	trigger = getTriggerMessageFromEEPROM();
	g_EmuMaster = getEmulatedMasterFromEEPROM();
	g_Emulating = FALSE;  

    USART_putCString( PSTR("\r\n\t\t           IEBus Controller\r\n") );
    USART_putCString( PSTR("\t\t   Copyright (C) 2008, Robert Newton\r\n") );
    USART_putCString( PSTR("\t\t          www.angrycamel.com      \r\n") );
    sprintf( USART_txBuffer, "\t\t      Firmware %s, %s\r\n\r\n", FIRMWARE_VERSION, FIRMWARE_DATE );
    USART_putString( USART_txBuffer );
    
    IEBus_Message txMsg;

    while ( 1 )
    {
        //Reset watchdog.
        wdt_reset();
        
		IEBus_Message msg;
		IEBus_Error err;
		IEBus_Error txErr;
 
		//Init the message and error structs to 0
		memset (&msg, 0, sizeof( msg ));
		memset (&err, 0, sizeof( err ));
		memset (&txErr, 0, sizeof( txErr ));

		//===================================================//
		// Get the current message from the bus              //
		//===================================================//
		err = IEBus_getMessage(&msg, g_Emulating, g_EmuMaster);

		if ( err.result == TRUE ) {
			listenForModeSwitch(&msg);
			sendMessageOverSerial(&msg);
		}else{
			sendErrorOverSerial(&err);
		}

		//===================================================//
		// Listen for any commands being sent from the PC    //
		//===================================================//
		//DISABLED!!!
		if ( USART_rxByteCount < 0 ){  //Change to ">" to enable
			cbi(UCSRB, RXCIE);
			if(getRxCommand( &txMsg )){
				//Put the formatted IEBus message onto the bus
				txErr = IEBus_putMessage(&txMsg);

				//Check for failure and if so report it to the PC
				if ( txErr.result != TRUE ){
					sendErrorOverSerial(&err);
				}
				
				//Output a human readable form of the message over serial
				// for debugging purposes. This should not stay in once
				// we know everything works. (This will not work in Studio)
				USART_putCString( PSTR("\r\n================================\r\n") );
				sprintf(USART_txBuffer, "broadcast: 0x%.1X\r\nmaster: 0x%.4X\r\nslave: 0x%.4X\r\ncontrol: 0x%.2X\r\ndatasize: %d\r\n", txMsg.broadcast, txMsg.master, txMsg.slave, txMsg.control, txMsg.datasize);
				USART_putString(USART_txBuffer);
				for (short i=0;i<txMsg.datasize;i++) {
					sprintf(USART_txBuffer, "data[%d]: 0x%.2X\r\n",i, txMsg.data[i]);
					USART_putString(USART_txBuffer);
				}
				USART_putCString( PSTR("================================\r\n") );
			}
			sbi(UCSRB, RXCIE);
		}
    }
}

/*--------------------------------------------------------------------------------------------------
                                            ISRs.
--------------------------------------------------------------------------------------------------*/
//Interupt when recieving a byte, in order to add it to the rx buffer.
ISR(USART_RXC_vect)
{
    USART_rxBuffer[USART_rxByteCount] = UDR; 
    USART_rxByteCount++;
}

/*--------------------------------------------------------------------------------------------------
                                         End of file.
--------------------------------------------------------------------------------------------------*/
