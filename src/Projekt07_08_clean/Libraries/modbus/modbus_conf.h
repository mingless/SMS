#ifndef __MODBUS_CONF_H
#define __MODBUS_CONF_H

#define MB_MASTER 																	1
#define MB_SLAVE 																		( MB_MASTER == 0 )
#define MB_SLAVE_ADDRESS 														102
#define MB_BROADCAST_ADDRESS 												0x00

#define DISCRETES_INPUT_ENABLED				        			0
#define DISCRETES_INPUT_START 											0
#define DISCRETES_INPUT_NREGS 											1

#define COILS_ENABLED 															0
#define COILS_START 																0
#define COILS_NREGS 																4

#define DISCRETES_INPUT_MATCHES_COILS		 						0


#define INPUT_REGISTERS_ENABLED						 					0
#define INPUT_REGISTERS_START 											0
#define INPUT_REGISTERS_NREGS 											1	

#define HOLDING_REGISTERS_ENABLED 									0
#define HOLDING_REGISTERS_START											0
#define HOLDING_REGISTERS_NREGS 										1	

#define INPUT_REGISTERS_MATCHES_HOLDING_REGISTERS		0

#define MB_BUF_SIZE_MAX															1024
#define MB_MSEC_MUL																	100

#endif
