/*--------------------------------------------------------------------------------------------------

  Name         :  eeprom.h

  Description  :  Class to wrap methods for writing to the internal eeprom

  Author       :  Robert Newton (robbienewton@gmail.com)
  
  Copyright    :  (c) 2007 Robert Newton
  
--------------------------------------------------------------------------------------------------*/

#ifndef _EEPROM_H_
#define _EEPROM_H_

/*--------------------------------------------------------------------------------------------------
                                         Prototypes
--------------------------------------------------------------------------------------------------*/
bool		    EEPROM_getValue		( byte* value, byte* addr, word byteCount );
void		    EEPROM_putValue		( byte* value, byte* addr, word byteCount );

#endif // _EEPROM_H_

