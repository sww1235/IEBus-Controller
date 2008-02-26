/*--------------------------------------------------------------------------------------------------

  Name         :  eeprom.c

  Description  :  Class to wrap methods for writing to the internal eeprom

  Author       :  Based on Procyon AVRlib eeprom methods
  
  Copyright    :  (c) 2007 Robert Newton
  
--------------------------------------------------------------------------------------------------*/

#include <avr/eeprom.h>

#include "Globals.h"

/*--------------------------------------------------------------------------------------------------
                                      Global Variables
--------------------------------------------------------------------------------------------------*/
static const char * Description;

/*--------------------------------------------------------------------------------------------------
                                          Methods
--------------------------------------------------------------------------------------------------*/
bool EEPROM_getValue(byte* value, byte* addr, word byteCount)
{
    word i;
    byte checksum_stored = 0;
    byte checksum = 0;

    // load value
    eeprom_read_block(value, addr, byteCount);
    
    // load checksum
    eeprom_read_block(&checksum_stored, addr+byteCount, sizeof(byte));

    // calculate own checksum
    for(i=0;i<byteCount;i++)
        checksum += value[i];
    checksum = ~checksum;
    
    if(checksum == checksum_stored)
        return TRUE;
    else
        return FALSE;
}

void EEPROM_putValue(byte* value, byte* addr, word byteCount)
{
    word i;
    byte checksum = 0;

    // calculate checksum
    for(i=0;i<byteCount;i++)
        checksum += value[i];
    checksum = ~checksum;

    // store parameters
    eeprom_write_block(value, addr, byteCount);
    // store checksum
    eeprom_write_block(&checksum, addr+byteCount, sizeof(byte));
}