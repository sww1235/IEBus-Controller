/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus.c

  Description  :  Inter Equipment Bus™ controller class
                  Based on code by Louis Frigon (info@sigmaobjects.com)

  Author       :  Robert Newton (robbienewton@gmail.com)
  
  Copyright    :  (c) 2007 Robert Newton
  
--------------------------------------------------------------------------------------------------*/
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h> 
#include <avr/io.h>
#include <stdio.h>

#include "Globals.h"
#include "USART.h"
#include "IEBus.h"
#include "eeprom.h"

/*--------------------------------------------------------------------------------------------------
                                      Local Functions
--------------------------------------------------------------------------------------------------*/
//Receive
static word         IEBus_getBits ( byte bitCount );
//static byte         IEBus_getBits ( byte bitCount );
static void         IEBus_getACK ( void );

//Transmit
static bool         IEBus_isAvailable ( void );
static void         IEBus_putStartBit ( void );
static void         IEBus_put1Bit ( bool );
static void         IEBus_put4Bits ( byte );
static void         IEBus_put8Bits ( byte );
static void         IEBus_put12Bits ( word );
static bool         IEBus_doACK ( bool b );

/*--------------------------------------------------------------------------------------------------
                                      Global Variables
--------------------------------------------------------------------------------------------------*/
static bool         ParityBit;







/*--------------------------------------------------------------------------------------------------
                                          Configuration
----------------------------------------------------------------------------------------------------

  Name         :  IEBus_setMasterFilter

  Description  :  Sets the master address to filter by into the eeprom

  Argument(s)  :  word (masterAddress) -> The master address to filter by
  
  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void IEBus_setMasterFilter( word masterAddress )
{
	//Always store the master addres filter at the first position in eeprom
	byte addr = 0;  

	//Take the passed in master address and set it in the eeprom
	//EEPROM_putValue ( &masterAddress, &addr, 2 );
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_getMasterFilter

  Description  :  Retrieves the master address filter from the eeprom

  Argument(s)  :  None.
  
  Return value :  Word (masterAddress)

--------------------------------------------------------------------------------------------------*/
word IEBus_getMasterFilter( void )
{
	//Always store the master addres filter at the first position in eeprom
	byte addr = 0;  
	word masterAddress;

	//Take the passed in master address and set it in the eeprom
	//if(EEPROM_getValue ( (byte *) &masterAddress, &addr, 2 ))
	//	return masterAddress;
	//else
	//	return 0;
    return 0;
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_setSlaveFilter

  Description  :  Sets the slave address to filter by into the eeprom

  Argument(s)  :  word (slaveAddress) -> The slave address to filter by
  
  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void IEBus_setSlaveFilter( word slaveAddress )
{
	//Always store the master addres filter at the first position in eeprom
	byte addr = 0;  

	//Take the passed in master address and set it in the eeprom
	//EEPROM_putValue ( &slaveAddress, &addr, 2 );
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_getSlaveFilter

  Description  :  Retrieves the slave address filter from the eeprom

  Argument(s)  :  None.
  
  Return value :  Word (slaveAddress)

--------------------------------------------------------------------------------------------------*/
word IEBus_getSlaveFilter( void )
{
	//Always store the master addres filter at the first position in eeprom
	byte addr = 0;  
	word slaveAddress;

	//TODO: Figure out why this isn't working
	//Take the passed in master address and set it in the eeprom
	//if(EEPROM_getValue ( &slaveAddress, &addr, 2 ))
	//	return slaveAddress;
	//else
		return 0;
}












/*--------------------------------------------------------------------------------------------------
                                               Misc
----------------------------------------------------------------------------------------------------

  Name         :  IEBus_cmpMessage

  Description  :  Compare the contents of two rx message structs and return true if all matches

  Argument(s)  :  Two IEBus_Message's to compare to each other
  
  Return value :  Boolean - return true if the two messages match

--------------------------------------------------------------------------------------------------*/

bool IEBus_cmpMessage	( IEBus_Message *msg1, IEBus_Message *msg2 )
{	
	/*
	USART_putCString( PSTR("\r\n\r\n\r\nCompare Two IEBus Messages\r\n================================") );	

	USART_putCString( PSTR("\r\nbroadcast: ") );
	if  (msg1->broadcast	==		msg2->broadcast	){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	} 
	USART_putCString( PSTR("\r\nmaster: ") );
	if  (msg1->master			==		msg2->master		){
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\nslave: ") );
	if  (msg1->slave			==		msg2->slave			){
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}		  
	USART_putCString( PSTR("\r\ncontrol: ") );
	if  (msg1->control		==		msg2->control		){
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndatasize: ") );
	if  (msg1->datasize		==		msg2->datasize	){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[0]: ") );
	if  (msg1->data[0]		==		msg2->data[0]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[1]: ") );
	if  (msg1->data[1]		==		msg2->data[1]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[2]: ") );
	if  (msg1->data[2]		==		msg2->data[2]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\nXdata[3]: ") );
	if  (msg1->data[3]		==		msg2->data[3]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[4]: ") );
	if  (msg1->data[4]		==		msg2->data[4]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[5]: ") );
	if  (msg1->data[5]		==		msg2->data[5]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[6]: ") );
	if  (msg1->data[6]		==		msg2->data[6]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[7]: ") );
	if  (msg1->data[7]		==		msg2->data[7]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[8]: ") );
	if  (msg1->data[8]		==		msg2->data[8]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[9]: ") );
	if  (msg1->data[9]		==		msg2->data[9]		){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[10]: ") );
	if  (msg1->data[10]		==		msg2->data[10]	){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[11]: ") );
	if  (msg1->data[11]		==		msg2->data[11]	){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[12]: ") );
	if  (msg1->data[12]		==		msg2->data[12]	){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[13]: ") );
	if  (msg1->data[13]		==		msg2->data[13]	){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[14]: ") );
	if  (msg1->data[14]		==		msg2->data[14]	){	
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}	
	USART_putCString( PSTR("\r\ndata[15]: ") );
	if  (msg1->data[15]		==		msg2->data[15]	){
		USART_putCString( PSTR("Matches") );
	}else{
		USART_putCString( PSTR("Doesn't match!") );
	}
	*/
	
	return (
		(msg1->broadcast	==		msg2->broadcast	) &&	
		(msg1->master		==		msg2->master	) &&			
		(msg1->slave		==		msg2->slave		) &&		  
		(msg1->control		==		msg2->control	) &&		
		(msg1->datasize		==		msg2->datasize	) &&		
		(msg1->data[0]		==		msg2->data[0]	) &&		
		(msg1->data[1]		==		msg2->data[1]	) &&		
		(msg1->data[2]		==		msg2->data[2]	) &&		
		(msg1->data[3]		==		msg2->data[3]	) &&		
		(msg1->data[4]		==		msg2->data[4]	) &&		
		(msg1->data[5]		==		msg2->data[5]	) &&		
		(msg1->data[6]		==		msg2->data[6]	) &&		
		(msg1->data[7]		==		msg2->data[7]	) &&		
		(msg1->data[8]		==		msg2->data[8]	) &&		
		(msg1->data[9]		==		msg2->data[9]	) &&		
		(msg1->data[10]		==		msg2->data[10]	) &&		
		(msg1->data[11]		==		msg2->data[11]	) &&		
		(msg1->data[12]		==		msg2->data[12]	) &&		
		(msg1->data[13]		==		msg2->data[13]	) &&		
		(msg1->data[14]		==		msg2->data[14]	) &&		
		(msg1->data[15]		==		msg2->data[15]	)
	);
}











/*--------------------------------------------------------------------------------------------------
                                              Recieve
----------------------------------------------------------------------------------------------------

  Name         :  IEBus_getBits

  Description  :  Receive up to 12 bits from the IEBus.

  Argument(s)  :  byte (bitCount) -> The number of bits to retrieve (limited by 12 bit return).
  
  Return value :  (word) -> 12 bit var containing the bits read from the IEBus

--------------------------------------------------------------------------------------------------*/
word IEBus_getBits ( byte bitCount )
{
    word data = 0;
    ParityBit = 0;

    while (bitCount-- > 0){
        data <<= 1;

        // Wait until rising edge of new bit.
        while(INPUT_IS_CLEAR){
            wdt_reset();
        }

        TCNT0 = 0;
        while(INPUT_IS_SET);
        
        // Compare half way between a '1' (20 us) and a '0' (32 us ): 32 - (32 - 20) /2 = 26 us
        if (TCNT0 < BIT_0_HOLD_ON_LENGTH - (BIT_0_HOLD_ON_LENGTH - BIT_1_HOLD_ON_LENGTH) / 2){
            data |= 0x0001;
            ParityBit = ! ParityBit;
        }
    }
    return data;
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_getACK

  Description  :  Simply read 1 bit (but this makes the source easier to read

  Argument(s)  :  None.

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void IEBus_getACK ( void )
{
	//This makes reading the code easier
    IEBus_getBits(1);
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_getMessage

  Description  :  Get the current message being transmitted over the IEBus
                  If the sniffer flag is true then simply listen for the ACK's, else send the
				  ACK's yourself.

  Argument(s)  :  IEBus Message Container, Sniffing Enabled Flag

  Return value :  (bool) -> TRUE if it was successful

--------------------------------------------------------------------------------------------------*/
IEBus_Error IEBus_getMessage ( IEBus_Message *msg, bool emulating, word emuMaster )
{
	IEBus_Error err;
	memset (&err, 0, sizeof( err ));

    activityIndicatorOn();

	//Recieve Header
    IEBus_getBits( 1 );													// Start bit
	msg->broadcast = IEBus_getBits(1);									// Broadcast bit
	msg->master = IEBus_getBits(12);									// Master Address
    if ( ParityBit != IEBus_getBits(1) )	{							// Parity
		err.result = FALSE;
		err.code = 1;
		strcpy(err.description, "Parity error getting Master Address");
		activityIndicatorOff();
		return err;
	}
    msg->slave = IEBus_getBits(12);										// Slave Address
    if ( ParityBit != IEBus_getBits(1) )	{							// Parity
		err.result = FALSE;
		err.code = 2;
		strcpy(err.description, "Parity error getting Slave Address");
		activityIndicatorOff();
		return err;
	}

	//Get/Send ACK for the Header
	if ( emulating && (emuMaster == msg->master) ) {
		IEBus_getACK();														// ACK
	}else{
		IEBus_doACK(msg->broadcast);
	}

	//Recieve Control
    msg->control = IEBus_getBits(4);									// Control
    if ( ParityBit != IEBus_getBits(1) )	{							// Parity
		err.result = FALSE;
		err.code = 3;
		strcpy(err.description, "Parity error getting Control");
		activityIndicatorOff();
		return err;
	}

	//Get/Send ACK for the Control
	if ( emulating && (emuMaster == msg->master) ) {
		IEBus_getACK();														// ACK
	}else{
		IEBus_doACK(msg->broadcast);
	}

	//Recieve Telegraph length (DataSize)
    msg->datasize = IEBus_getBits(8);									// DataSize
    if ( ParityBit != IEBus_getBits(1) )	{							// Parity
		err.result = FALSE;
		err.code = 4;
		strcpy(err.description, "Parity error getting DataSize");
		activityIndicatorOff();
		return err;
	}

	//Get/Send ACK for the DataSize
	if ( emulating && (emuMaster == msg->master) ) {
		IEBus_getACK();														// ACK
	}else{
		IEBus_doACK(msg->broadcast);
	}

	//Recieve Data
    for(byte i=0; i<msg->datasize; i++ ){								// LOOP (DataSize)
		msg->data[i] = IEBus_getBits(8);									// Data
		if ( ParityBit != IEBus_getBits(1) )	{							// Parity
			err.result = FALSE;
			err.code = 5;
			strcpy(err.description, "Parity error getting data");
			activityIndicatorOff();
			return err;
		}
		//Get/Send ACK for the current data block
		if ( emulating && (emuMaster == msg->master) ) {
			IEBus_getACK();														// ACK
		}else{
			IEBus_doACK(msg->broadcast);
		}
    }																	// END LOOP

	activityIndicatorOff();

	err.result = TRUE;
	err.code = 2;
	return err;  
}
















/*--------------------------------------------------------------------------------------------------
                                              Transmit
----------------------------------------------------------------------------------------------------

  Name         :  IEBus_put1Bit

  Description  :  Transmits 1 bit passed in as a parameter onto the IEBus.

  Argument(s)  :  bool (data) -> 8 bit var, but only transmit the first bit.
  
  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void IEBus_put1Bit ( bool data )
{
    TCNT0 = 0;
    DDRD |= _BV(PD2) | _BV(PD3);
    
    if(data){
        while(TCNT0 < BIT_1_HOLD_ON_LENGTH);
    }else{
        while(TCNT0 < BIT_0_HOLD_ON_LENGTH);
    }
    
    DDRD &= ~( _BV(PD2) | _BV(PD3) );
    while ( TCNT0 <  NORMAL_BIT_LENGTH );
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_put4Bits

  Description  :  Transmits 4 bits passed in as a parameter onto the IEBus.

  Argument(s)  :  byte (data) -> 8 bit var, but only transmit the first 4 bits.
  
  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void IEBus_put4Bits ( byte data )
{
    ParityBit = 0;
    
    for(char bitCount=0; bitCount<4; bitCount++){
        TCNT0 = 0;
        DDRD |= _BV(PD2) | _BV(PD3);
        
        if (data & 0x8){
            ParityBit = ! ParityBit;
            while(TCNT0 < BIT_1_HOLD_ON_LENGTH);
        }else{
            while(TCNT0 < BIT_0_HOLD_ON_LENGTH);
        }
        
        DDRD &= ~( _BV(PD2) | _BV(PD3) );
        while ( TCNT0 < NORMAL_BIT_LENGTH );
        data <<= 1;
    }   
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_put8Bits

  Description  :  Transmits 8 bits passed in as a parameter onto the IEBus.

  Argument(s)  :  byte (data) -> 8 bit var, and transmit all 8 bits.
  
  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void IEBus_put8Bits ( byte data )
{
    ParityBit = 0;
    
    for(char bitCount=0; bitCount<8; bitCount++){
        TCNT0 = 0;
        DDRD |= _BV(PD2) | _BV(PD3);
        
        if (data & 0x80){
            ParityBit = ! ParityBit;
            while(TCNT0 < BIT_1_HOLD_ON_LENGTH);
        }else{
            while(TCNT0 < BIT_0_HOLD_ON_LENGTH);
        }
        
        DDRD &= ~( _BV(PD2) | _BV(PD3) );
        while ( TCNT0 < NORMAL_BIT_LENGTH );
        data <<= 1;
    }   
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_put12Bits

  Description  :  Transmits 12 bits passed in as a parameter onto the IEBus.

  Argument(s)  :  word (data) -> 16 bit var, but only transmit the first 12 bits.
  
  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void IEBus_put12Bits ( word data )
{
    ParityBit = 0;
       
    for(char bitCount=0; bitCount<12; bitCount++){
        TCNT0 = 0;
        DDRD |= _BV(PD2) | _BV(PD3);
        
        if(data & 0x0800){
            ParityBit = ! ParityBit;
            while(TCNT0 < BIT_1_HOLD_ON_LENGTH);
        }else{
            while(TCNT0 < BIT_0_HOLD_ON_LENGTH);
        }
        
        DDRD &= ~( _BV(PD2) | _BV(PD3) );
        while ( TCNT0 < NORMAL_BIT_LENGTH );
        data <<= 1;
    }   
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_isAvailable

  Description  :  Determine whether the bus is free (no tx/rx).

  Argument(s)  :  None.

  Return value :  (bool) -> TRUE the bus is available for rx or tx.

--------------------------------------------------------------------------------------------------*/
bool IEBus_isAvailable ( void )
{
    TCNT0 = 0;
    while(INPUT_IS_CLEAR){
        // We assume the bus is free if nothing happens for the length of 1 bit.
        if(TCNT0 > NORMAL_BIT_LENGTH){ return TRUE; }
    }
    return FALSE;
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_putStartBit

  Description  :  Transmits the IEBus standard start bit onto the IEBus.

  Argument(s)  :  None.
  
  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void IEBus_putStartBit ( void )
{
    TCNT0 = 0;
    DDRD |= _BV(PD2) | _BV(PD3);
    while ( TCNT0 < START_BIT_HOLD_ON_LENGTH );
    DDRD &= ~( _BV(PD2) | _BV(PD3) );
    while ( TCNT0 < START_BIT_LENGTH );
}

/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus_putMessage

  Description  :  Transmit an IEBus message onto the IEBus.

  Argument(s)  :  IEBus_Message (msg *) -> The message to be sent over the IEBus.
  
  Return value :  (IEBus_Error) -> IEBus error structure (IEBus_Error.result = TRUE for success)

--------------------------------------------------------------------------------------------------*/
IEBus_Error IEBus_putMessage ( IEBus_Message *msg )
{  
	IEBus_Error err;

    while(!IEBus_isAvailable());

    activityIndicatorOn();
    
	//Transmit header
    IEBus_putStartBit();
    IEBus_put1Bit(msg->broadcast);
    IEBus_put12Bits(msg->master);
    IEBus_put1Bit(ParityBit);
    IEBus_put12Bits(msg->slave);
    IEBus_put1Bit(ParityBit);
	if(!IEBus_doACK(msg->broadcast))
	{
		err.result = FALSE;
		err.code = 1;
		strcpy(err.description, "Header did not ACK");
		activityIndicatorOff();
		return err;
	}
    
	//Transmit control
    IEBus_put4Bits(msg->control);
    IEBus_put1Bit(ParityBit);
    if(!IEBus_doACK(msg->broadcast))
	{
		err.result = FALSE;
		err.code = 2;
		strcpy(err.description, "Control did not ACK");
		activityIndicatorOff();
		return err;
	}
    
	//Transmit telegraph length (datasize)
	IEBus_put8Bits(msg->datasize);
    IEBus_put1Bit(ParityBit);
    if(!IEBus_doACK(msg->broadcast))
	{
		err.result = FALSE;
		err.code = 3;
		strcpy(err.description, "DataSize did not ACK"); 
		activityIndicatorOff();
		return err;
	}
    
	//Transmit data
    for(byte i=0; i < msg->datasize; i++){
		IEBus_put8Bits(msg->data[i]);
		IEBus_put1Bit(ParityBit);
		if(!IEBus_doACK(msg->broadcast))
		{
			err.result = FALSE;
			err.code = 4;
			sprintf( err.description, "Data did not ACK ( data[%d] = 0x%X )", i, msg->data[i] );
			activityIndicatorOff();
			return err;
		}
	}
    
    activityIndicatorOff();

	err.result = TRUE;
	return err;
}

bool IEBus_doACK ( bool b )
{
    if(b == IS_BROADCAST){   
        // Acknowledge.    
        IEBus_put1Bit(0);
        return TRUE;
    }

    // Return acknowledge bit.
    // The acknowledge pattern is very tricky: the sender shall drive the bus for the equivalent
    // of a bit '1' (20 us) then release the bus and listen. At this point the target shall have
    // taken over the bus maintaining the pulse until the equivalent of a bit '0' (32 us) is formed.
    TCNT0 = 0;
    DDRD |= _BV(PD2) | _BV(PD3);
    while(TCNT0 < BIT_1_HOLD_ON_LENGTH);
    DDRD &= ~( _BV(PD2) | _BV(PD3) );
    
    // Measure final resulting bit.
    while(INPUT_IS_SET);
    
    // Sample half-way through bit '0' (26 us) to detect whether the target is acknowledging.
    if(TCNT0 > BIT_0_HOLD_ON_LENGTH - (BIT_0_HOLD_ON_LENGTH - BIT_1_HOLD_ON_LENGTH) / 2){
        // Slave is acknowledging (ack = 0). Wait until end of ack bit.
        while ( INPUT_IS_SET );
        return TRUE;
    }
    
    // No sign of life on the bus.
    return FALSE; 
}    
