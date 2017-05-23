#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdbool.h>
#include "modbus_conf.h"
#include "crc16.h"

/* Typedefs -----------------------------------------------------------*/
typedef enum{STATE_START, // additional state
						 STATE_INITIAL, 
						 STATE_IDLE, 
						 STATE_EMISSION, 
						 STATE_RECEPTION, 
						 STATE_CONTROL_AND_WAITING
} MB_STATE; // MODBUS over serial line specification and implementation guide V1.0, p: 14, Fig. 14

typedef enum{FUN_READ_DISCRETE_INPUTS											= 0x02,
						 FUN_READ_COILS 															= 0x01,
						 FUN_WRITE_SINGLE_COIL												= 0x05,
						 FUN_WRITE_MULTIPLE_COILS											= 0x0F,
						 FUN_READ_INPUT_REGISTER											= 0x04,
						 FUN_READ_HOLDING_REGISTER										= 0x03,
						 FUN_WRITE_SINGLE_REGISTER										= 0x06,
						 FUN_WRITE_MULTIPLE_REGISTERS									= 0x10,
						 FUN_READ_WRITE_MULTIPLE_REGISTERS						= 0x17,
						 FUN_MASK_WRITE_REGISTER											= 0x16,
						 FUN_READ_FIFO_QUEUE													= 0x18,
} MB_FUNCTION; // MODBUS Application Protocol Specification V1.1b3, p: 11

typedef enum{ERR_ILLEGAL_FUNCTION													= 0x01,
						 ERR_ILLEGAL_DATA_ADDRESS											= 0x02,
						 ERR_ILLEGAL_DATA_VALUE												= 0x03,
						 ERR_SERVER_DEVICE_FAILURE										= 0x04,
						 ERR_ACKNOWLEDGE															= 0x05,
						 ERR_SERVER_DEVICE_BUSY												= 0x06,
						 ERR_MEMORY_PARITY_ERROR											= 0x08,
						 ERR_GATEWAY_PATH_UNAVAILABLE									= 0x0A,
						 ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND 	= 0x0B,
} MB_FUNCTION_ERROR; // MODBUS Application Protocol Specification V1.1b3, pp: 48-49

typedef enum{RESPONSE_OK,
						 RESPONSE_TIMEOUT,
						 RESPONSE_WRONG_ADDRESS,
						 RESPONSE_WRONG_FUNCTION,
						 RESPONSE_ERROR,
} MB_RESPONSE_STATE;

typedef enum{COIL_ON 																			= 0xFF00,
						 COIL_OFF 																		= 0x0000,
} MB_COIL_STATE;

bool MB_Buf_append(uint8_t ch);
uint8_t MB_Buf_pop(void);
bool MB_Buf_control(void);
void MB_Buf_process_slave(void);
void MB_Buf_process_master(void);
void MB_Buf_clear(void);

void Set50usTimer(uint32_t ticks35, uint32_t ticks15);
void Start50usTimer(void);
void Stop50usTimer(void);
void Reset50usTimer(void);
void SetTimeout(__IO uint32_t msec);
bool TimeoutPassed(void);
bool Ist15Expired(void);
bool Ist35Expired(void);
bool Is50usTimerStarted(void);
void TimeoutTick(void);

void MB_SendRequest(uint8_t addr, MB_FUNCTION f, uint8_t* datain, uint16_t lenin);
MB_RESPONSE_STATE MB_GetResponse(uint8_t addr, MB_FUNCTION f, uint8_t** dataout, uint16_t* lenout, uint32_t timeout);

extern void Communication_Put(uint8_t c); 					// to implement
extern uint8_t Communication_Get(void); 						// to implement
extern void Communication_Mode(bool rx, bool tx); 	// to implement
extern void Enable50usTimer(void); 									// to implement
extern void Disable50usTimer(void); 								// to implement

void SetCharacterReadyToTransmit(void); 	// set flag, that the character is ready to be transmitted
void SetCharacterReceived(bool); 					// set flag, that the character is available/no more available to read
void Timer50usTick(void); 								// increment the number of 50us ticks accumulated
void MB(void); 														// to call from time to time (each 10us)
void MB_Config(uint32_t baudrate); 				// to call at the beggining (to configure timer)

#endif
