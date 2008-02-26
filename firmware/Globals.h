/*--------------------------------------------------------------------------------------------------

  Name         :  Globals.h

  Description  :  Global definitions

  Author       :  Robert Newton (robbienewton@gmail.com)
  
  Copyright    :  (c) 2007 Robert Newton (angrycamel.com)
  
--------------------------------------------------------------------------------------------------*/
#ifndef _GLOBALS_H_
#define _GLOBALS_H_

#include <avr/io.h>

/*--------------------------------------------------------------------------------------------------
                                          Constants
--------------------------------------------------------------------------------------------------*/
#define FALSE                   0
#define TRUE                    (!FALSE)

// AVC LAN bus directly connected to internal analog comparator (PD6/7)
// PD6 AIN0 +
// PD7 AIN1 -
#define	DATAIN_PIN		        ACSR
#define DATAIN			        ACO

#define INPUT_IS_SET            ( bit_is_set( DATAIN_PIN, DATAIN ) )
#define INPUT_IS_CLEAR          ( bit_is_clear( DATAIN_PIN, DATAIN ) )

#define LED_DDR		            DDRC
#define LED_PORT	            PORTC
#define LEDOUT		            _BV(PORT0)
#define RELAYOUT	            _BV(PORT1)
#define RELAY2OUT	            _BV(PORT2)
#define LED2OUT		            _BV(PORT3)
#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))

 

/*--------------------------------------------------------------------------------------------------
                                       Type definitions
--------------------------------------------------------------------------------------------------*/
typedef char                    bool;     
typedef unsigned char           byte;
typedef unsigned short          word;

/*--------------------------------------------------------------------------------------------------
                                         Prototypes
--------------------------------------------------------------------------------------------------*/
inline void activityIndicatorOff( void );   
inline void activityIndicatorOn( void );     //Indicates that IEBus packets are being processed
inline void videoOverrideOff( void );
inline void videoOverrideOn( void );         //Flips the relay on to engage the video converter
inline void modeIndicatorOff ( void );
inline void modeIndicatorOn ( void );		 //Indicates either PC or Navi mode
inline void IEBusPassthroughOff ( void );
inline void IEBusPassthroughOn ( void );     //Flips the relay to cut off the bus to the navi comp

#endif   //  _GLOBALS_H_

/*--------------------------------------------------------------------------------------------------
                                         End of file.
--------------------------------------------------------------------------------------------------*/
