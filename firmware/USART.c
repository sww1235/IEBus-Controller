/*--------------------------------------------------------------------------------------------------

  Name         :  USART.c

  Description  :  Header file for USART functions.
                  Based on code by Louis Frigon (info@sigmaobjects.com)

  Author       :  Robert Newton (robbienewton@gmail.com)
  
  Copyright    :  (c) 2007 Robert Newton

--------------------------------------------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "USART.h"
#include "string.h"

/*--------------------------------------------------------------------------------------------------

  Name         :  USART_init

  Description  :  Performs USART initialization: 9600,N,8,1.

  Argument(s)  :  None.

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void USART_init ( void )
{
    //  Disable USART while setting baud rate.
    UCSRB = 0x00; 
    UCSRA = 0x00;
    
	// Turn on the transmission and reception circuitry
	UCSRB |= (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);   

    //  8 data bit, 1 stop, no parity.
    UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);

    //  Set USART baud rate @ 9600. Divider is 52 @ 8 MHz. 
    UBRRL = 52;
    
    // Load upper 8-bits of the baud rate value into the high byte of the UBRR register 
    UBRRH = ( 52 >> 8 );
  
    //  Enable internal pull-up on Rx pin.
    //PORTD |= _BV(PD0);
    
    USART_rxByteCount = 0;
}

/*--------------------------------------------------------------------------------------------------

  Name         :  USART_rxBufferClear

  Description  :  Empty out the USART Rx buffer and set the byte count to 0

  Argument(s)  :  None.

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void USART_rxBufferClear ( void )
{  
    memset (&USART_rxBuffer, 0, USART_BUFFER_SIZE);
	USART_rxByteCount = 0;
}

/*--------------------------------------------------------------------------------------------------

  Name         :  USART_isChar

  Description  :  Return status of USART Rx buffer.

  Argument(s)  :  None.

  Return value :  0 if Rx buffer empty.

--------------------------------------------------------------------------------------------------*/
bool USART_isChar ( void )
{  
    return UCSRA & _BV(RXC);
}

/*--------------------------------------------------------------------------------------------------

  Name         :  USART_getChar

  Description  :  Return character USART Rx buffer. Blocking until Rx buffer not empty.

  Argument(s)  :  None.

  Return value :  Character in Rx buffer.

--------------------------------------------------------------------------------------------------*/
char USART_getChar ( void )
{ 
    while ( !USART_isChar() );

    return UDR;
}

/*--------------------------------------------------------------------------------------------------

  Name         :  USART_putChar

  Description  :  Send a character through the USART.
  
  Argument(s)  :  c -> char to send.

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void USART_putChar ( char c )
{
    //  Wait for transmit register to be empty.
    while ( !(UCSRA & _BV(UDRE)) );
    
    UDR = c;
}

/*--------------------------------------------------------------------------------------------------

  Name         :  USART_putString

  Description  :  Transmit a string on the serial port.

  Argument(s)  :  str -> pointer to string to send.

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void USART_putString ( char *str )
{
    while ( *str )
    {   
        USART_putChar( *str++ );
    }
}

/*--------------------------------------------------------------------------------------------------

  Name         :  USART_putCString

  Description  :  Transmit a string on the serial port.

  Argument(s)  :  str -> pointer to constant string to send (strings in Flash).

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void USART_putCString ( const char *str )
{      
    char c;
    
    while ( (c = pgm_read_byte_near( str++ )) )
    {
        USART_putChar( c );
    }
}

/*--------------------------------------------------------------------------------------------------

  Name         :  USART_putHex

  Description  :  Transmit a string as a hex formatted byte array on the serial port.

  Argument(s)  :  str -> pointer to constant string to send (strings in Flash).

  Return value :  None.

--------------------------------------------------------------------------------------------------*/
void USART_putHex( const char *str )
{
	//Convert to hex then transmit
}

/*--------------------------------------------------------------------------------------------------
                                         End of file.
--------------------------------------------------------------------------------------------------*/

