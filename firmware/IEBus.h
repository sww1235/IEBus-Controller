/*--------------------------------------------------------------------------------------------------

  Name         :  IEBus.h

  Description  :  Inter Equipment Bus™ controller class
                  Based on code by Louis Frigon (info@sigmaobjects.com)

  Author       :  Robert Newton (robbienewton@gmail.com)
  
  Copyright    :  (c) 2007 Robert Newton
  
--------------------------------------------------------------------------------------------------*/

#ifndef _IEBUS_H_
#define _IEBUS_H_

#define NORMAL_BIT_LENGTH           37
#define BIT_1_HOLD_ON_LENGTH        20
#define BIT_0_HOLD_ON_LENGTH        32
#define START_BIT_LENGTH            186
#define START_BIT_HOLD_ON_LENGTH    168 
#define IS_BROADCAST				0

//Broadcast bit:
//   0 - broadcast
//   1 - normal
//Control values:
//   0x0 - Slave status (SSR) read
//   0x1 - Undefined
//   0x2 - Undefined
//   0x3 - Data read and lock
//   0x4 - Lock address read (Lower 8 bits)
//   0x5 - Lock address read (Upper 4 bits)
//   0x6 - Slave status (SSR) read and unlock
//   0x7 - Data read
//   0x8 - Undefined
//   0x9 - Undefined
//   0xA - Command write and lock
//   0xB - Data write and lock
//   0xC - Undefined
//   0xD - Undefined
//   0xE - Command write
//   0xF - Data write

typedef struct 
{
	bool				broadcast;			// Broadcast bit		[ 1 bit ]
	word				master;				// Master Address		[12 bits]
	word				slave;				// Slave Address		[12 bits]
    byte         		control;			// Control				[ 4 bits]
    byte                datasize;           // Data byte count		[ 8 bits]
    char                data[16];			// Data					[8n bits]
} IEBus_messageStruct;

typedef         IEBus_messageStruct IEBus_Message;

typedef struct 
{
	bool				result;				// Result (true=success, false=failed)
    byte         		code;				// Error code
    char                description[64];	// Description
} IEBus_errorStruct;

typedef         IEBus_errorStruct	IEBus_Error;

/*--------------------------------------------------------------------------------------------------
                                      Global Variables
--------------------------------------------------------------------------------------------------*/
bool            IEBus_filterMasterAddress;
bool            IEBus_filterSlaveAddress;

/*--------------------------------------------------------------------------------------------------
                                         Prototypes
--------------------------------------------------------------------------------------------------*/
IEBus_Error	IEBus_getMessage		( IEBus_Message *msg, bool emulating, word emuMaster );
IEBus_Error	IEBus_putMessage		( IEBus_Message *msg );

bool			IEBus_cmpMessage		( IEBus_Message *msg1, IEBus_Message *msg2 );
void            IEBus_setMasterFilter   ( word masterAddress );
word            IEBus_getMasterFilter   ( void );
void            IEBus_setSlaveFilter    ( word slaveAddress );
word            IEBus_getSlaveFilter    ( void );

#endif // _IEBUS_H_

