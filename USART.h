/*--------------------------------------------------------------------------------------------------

  Name         :  USART.h

  Description  :  Header file for USART functions.
                  Based on code by Louis Frigon (info@sigmaobjects.com)

  Author       :  Robert Newton (robbienewton@gmail.com)
  
  Copyright    :  (c) 2007 Robert Newton

--------------------------------------------------------------------------------------------------*/
#ifndef _USART_H_
#define _USART_H_

#include <avr/pgmspace.h>

#include "Globals.h"

#define USART_BUFFER_SIZE       128

/*--------------------------------------------------------------------------------------------------
                                      Global Variables
--------------------------------------------------------------------------------------------------*/

char            USART_txBuffer[ USART_BUFFER_SIZE ];
char            USART_rxBuffer[ USART_BUFFER_SIZE ];
char            USART_rxByteCount;

/*--------------------------------------------------------------------------------------------------
                                    Function prototypes
--------------------------------------------------------------------------------------------------*/
//  Function prototypes are mandatory otherwise the compiler generates unreliable code.

void			USART_init			( void );
bool			USART_isChar		( void );
char			USART_getChar		( void );
void			USART_putChar		( char c );
void			USART_putString		( char *str );
void			USART_putCString	( const char *str );
void			USART_rxBufferClear	( void );

#endif   //  _USART_H_
/*--------------------------------------------------------------------------------------------------
                                         End of file.
--------------------------------------------------------------------------------------------------*/
