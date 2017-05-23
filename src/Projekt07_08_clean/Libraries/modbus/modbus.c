#include "modbus.h"

bool DiscretesInputBuf[DISCRETES_INPUT_NREGS];
#if (DISCRETES_INPUT_MATCHES_COILS && COILS_ENABLED && DISCRETES_INPUT_ENABLED && (DISCRETES_INPUT_START) == COILS_START && (DISCRETES_INPUT_NREGS == COILS_NREGS) )
	#define CoilsBuf DiscretesInputBuf // create alias
#else
	bool CoilsBuf[COILS_NREGS]; // create new array
#endif

uint16_t InputRegistersBuf[INPUT_REGISTERS_NREGS];
#if (INPUT_REGISTERS_MATCHES_HOLDING_REGISTERS && INPUT_REGISTERS_ENABLED && HOLDING_REGISTERS_ENABLED && (INPUT_REGISTERS_START) == HOLDING_REGISTERS_START && (INPUT_REGISTERS_NREGS == HOLDING_REGISTERS_NREGS) )
	#define HoldingRegistersBuf InputRegistersBuf // create alias
#else
	uint16_t HoldingRegistersBuf[HOLDING_REGISTERS_NREGS];	 // create new array
#endif 
static __IO uint8_t MB_Buf[MB_BUF_SIZE_MAX] = {0}; // buffer for messages
static __IO uint16_t MB_Buf_size = 0;

static __IO bool character_recieved = false;
static __IO bool character_ready_to_send = false;
static __IO bool demand_of_emmision = false;
static __IO bool frame_ok = true;
static __IO bool response_ready = false;

__IO uint32_t timeout_time = 0;
__IO uint32_t t15_nominal_ticks = 0;
__IO uint32_t t35_nominal_ticks = 0;
__IO uint32_t _50us_ticks = 0;
__IO bool t15_expired = false;
__IO bool t35_expired = false;
__IO bool _50us_timer_started = false;

__IO uint16_t modbus_slave_address = 0;

static MB_STATE state = STATE_START;

void Timer50usTick(void){
	++_50us_ticks;
	if(_50us_ticks >= t15_nominal_ticks) t15_expired = true;
	if(_50us_ticks >= t35_nominal_ticks) t35_expired = true;
	if(t15_expired && t35_expired){
		Stop50usTimer();
	}
}

void SetTimeout(__IO uint32_t msec){
	timeout_time = msec*MB_MSEC_MUL;
}

void TimeoutTick(void){
	if(timeout_time > 0) 
		--timeout_time;
}

bool TimeoutPassed(void){
	if(timeout_time <= 0)
		return true;
	return false;
}

void Set50usTimer(uint32_t ticks35, uint32_t ticks15){
	Stop50usTimer();
	t15_expired = false;
	t35_expired = false;
	_50us_timer_started = false;
	t15_nominal_ticks = ticks15;
	t35_nominal_ticks = ticks35;
}

void Start50usTimer(void){
	_50us_ticks = 0;
	t15_expired = false;
	t35_expired = false;
	_50us_timer_started = true;
	Enable50usTimer();
}

void Stop50usTimer(void){
	_50us_timer_started = false;
	Disable50usTimer();
}

void Reset50usTimer(void){
	t15_expired = false;
	t35_expired = false;
	_50us_timer_started = false;
}

bool Ist15Expired(void){
	return t15_expired;
}

bool Ist35Expired(void){
	return t35_expired;
}
	
bool Is50usTimerStarted(void){
	return _50us_timer_started;
	
}

void SetCharacterReceived(bool b){ // to call
	character_recieved = b;
}

void SetCharacterReadyToTransmit(void){ // to call
	if(MB_Buf_size > 0)
		Communication_Put(MB_Buf_pop());
	else
		Communication_Mode(true, false);
}

void MB_SendRequest(uint8_t addr, MB_FUNCTION f, uint8_t* datain, uint16_t lenin){
	int i = 0;
	uint16_t crc = 0;
	while(state != STATE_IDLE || demand_of_emmision); // waiting for IDLE state
	MB_Buf_clear();
	modbus_slave_address = addr;
	MB_Buf[MB_Buf_size++] = addr;
	MB_Buf[MB_Buf_size++] = f;
	for(i = 0; i < lenin; ++i)
		MB_Buf[MB_Buf_size++] = datain[i];
	crc = CRC16((uint8_t*)MB_Buf, MB_Buf_size);
	MB_Buf[MB_Buf_size++] = crc&0xFF;
	MB_Buf[MB_Buf_size++] = crc>>8;
	demand_of_emmision = true;
	response_ready = false;
}

MB_RESPONSE_STATE MB_GetResponse(uint8_t addr, MB_FUNCTION f, uint8_t** dataout, uint16_t* lenout, uint32_t timeout){
	SetTimeout(timeout);
	while(!response_ready && !TimeoutPassed());
	
	*dataout = (uint8_t*)MB_Buf+2;
	if((MB_Buf_size - 2 /*ADDR,FUN*/ - 2 /*CRC*/) < 0) *lenout = 0;
	else *lenout = MB_Buf_size - 2 /*ADDR,FUN*/ - 2 /*CRC*/;
	
	if(TimeoutPassed()) return RESPONSE_TIMEOUT;
	if(addr != MB_Buf[0]) return RESPONSE_WRONG_ADDRESS;
	if(f != ((MB_FUNCTION) MB_Buf[1])){ 
		if(f+0x80 == ((MB_FUNCTION) MB_Buf[1]))
			return RESPONSE_ERROR;
		else
			return RESPONSE_WRONG_FUNCTION;
	}
	
	return RESPONSE_OK;
}

void MB_Config(uint32_t baudrate){
	if(baudrate > 19200)
		Set50usTimer(35, 15);
	else	
		Set50usTimer((7UL*220000UL)/(2UL*baudrate), (3UL*220000UL)/(2UL*baudrate));
	Stop50usTimer();
}

void MB(void){
	switch(state){
		case STATE_START: // starting timer
			Start50usTimer();
			Communication_Mode(true, false);
			state = STATE_INITIAL;
			break;
		case STATE_INITIAL: // waiting for t3.5
			if(Ist35Expired()){
				Stop50usTimer();
				Reset50usTimer();
				state = STATE_IDLE; // go to the next state
			} else if(character_recieved){
				Communication_Get();
				Start50usTimer(); // restart timer
			} else {
				// keep waiting
			}
			break;
		case STATE_IDLE:
			if(demand_of_emmision){ // first we send, ...
				Communication_Mode(false, false);
				state = STATE_EMISSION;
			} else if (character_recieved){        // ... then we receive
				MB_Buf_append(Communication_Get());
				Start50usTimer();
				state = STATE_RECEPTION;				
			} else {
				// keep waiting
			}
			break;
		case STATE_EMISSION:
			demand_of_emmision = false;
			if(Ist35Expired()){
				Reset50usTimer();
				state = STATE_IDLE;
				response_ready = false;
				MB_Buf_clear();
				Communication_Mode(true, false);
			} else if (MB_Buf_size == 0 && !Is50usTimerStarted()){
				Start50usTimer();
			} else if (MB_Buf_size > 0){
				Communication_Mode(false, true);
			} else {
				// keep waiting
			}
			break;
		case STATE_RECEPTION:
			if(Ist15Expired()){
				state = STATE_CONTROL_AND_WAITING; // do not stop timers!
				frame_ok = true;
			} else if(character_recieved){
				MB_Buf_append(Communication_Get());
				Start50usTimer();
			}	 else {
				// keep waiting
			}
			break;
		case STATE_CONTROL_AND_WAITING:
			frame_ok = MB_Buf_control();
			if(character_recieved && !Ist35Expired()){
				frame_ok = false;
			} else if (Ist35Expired()){
				Reset50usTimer();
				if(frame_ok){ // if frame OK  -> processing frame
					#if MB_MASTER
						MB_Buf_process_master();
					#else
						MB_Buf_process_slave();
					#endif
					response_ready = true;
				} else { 			// if frame NOK -> delete entire frame
					MB_Buf_clear();
				}
				state = STATE_IDLE;
			} else {
				// keep waiting
			}
			break;
		default:
			Communication_Mode(false, false);
			while(1); // something went terribly wrong
	}
}

bool MB_Buf_append(uint8_t ch){
	if(MB_Buf_size >= MB_BUF_SIZE_MAX)
		return false;
	MB_Buf[MB_Buf_size++] = ch;
	return true;
}

uint8_t MB_Buf_pop(void){
	uint8_t tmp;
	int i = 0;
	bool full = false;
	if(MB_Buf_size == MB_BUF_SIZE_MAX){
		full = true;
		--MB_Buf_size;
	}
	if(MB_Buf_size == 0){
		return 0;
	}
	tmp = MB_Buf[0];
	for(i=0; i<MB_Buf_size;++i) // from 0 to max 1023
		MB_Buf[i] = MB_Buf[i+1];
	if(full) MB_Buf[MB_Buf_size] = 0; // if MB_Buf_size == MB_Buf_size_MAX, we have to perform last assignment ourself
	else --MB_Buf_size;               // if full, then MB_Buf_size is already decremented, otherwise decrement now
	return tmp;
}

bool MB_Buf_control(void){
	#if MB_MASTER
		if(MB_Buf[0] != modbus_slave_address && MB_Buf[0] != MB_BROADCAST_ADDRESS) 
			return false;
	#endif
	if(CRC16((uint8_t*)MB_Buf, MB_Buf_size) != 0)	return false;
	return true;
}

void MB_Buf_process_master(void){
}

void MB_Buf_process_slave(void){
	// process
	int i = 0;
	uint8_t addr = MB_Buf[0];
	MB_FUNCTION func = (MB_FUNCTION) MB_Buf[1];
	bool error = false;
	uint16_t start;
	uint16_t quantity;
	uint16_t value;
	uint16_t crc;
	switch(func){
		case FUN_READ_COILS: // read coil
			start 	 = (MB_Buf[2]<<8)+MB_Buf[3];
			quantity = (MB_Buf[4]<<8)+MB_Buf[5];
		
			MB_Buf_clear(); // clear for next message
			if( !COILS_ENABLED ){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_FUNCTION;	
			} else if(quantity <= 0 || quantity > 2000){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_VALUE;		
			} else if(start+0 < COILS_START || start+quantity > COILS_START+COILS_NREGS){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_ADDRESS;		
			} else {
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				
				MB_Buf[MB_Buf_size++] = quantity/8+((quantity%8)==0?0:1);				
				for(i=0; i<quantity;){
					MB_Buf[MB_Buf_size] += (CoilsBuf[start+i]?1:0)<<(i%8);
					++i;
					if(i%8==0 && i!=0) ++MB_Buf_size;
				}
				if(quantity%8 != 0) ++MB_Buf_size;	
				
				if ( error ){
					MB_Buf_clear();
					MB_Buf[MB_Buf_size++] = addr;
					MB_Buf[MB_Buf_size++] = func+0x80;
					MB_Buf[MB_Buf_size++] = ERR_SERVER_DEVICE_FAILURE;
				}
			}
			crc = CRC16((uint8_t*)MB_Buf, MB_Buf_size);
			MB_Buf[MB_Buf_size++] = crc&0xFF;
			MB_Buf[MB_Buf_size++] = crc>>8;
			demand_of_emmision = true;
			break;
			
		case FUN_WRITE_SINGLE_COIL: // write coil
			start 	 = (MB_Buf[2]<<8)+MB_Buf[3];
			value    = (MB_Buf[4]<<8)+MB_Buf[5];
		
			MB_Buf_clear(); // clear for next message
			if( !COILS_ENABLED ){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_FUNCTION;	
			} else if(value != COIL_ON && value != COIL_OFF){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_VALUE;		
			} else if(start+0 < COILS_START || start >= COILS_START+COILS_NREGS){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_ADDRESS;		
			} else {
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = start>>8;				
				MB_Buf[MB_Buf_size++] = start&0xFF;				
				MB_Buf[MB_Buf_size++] = value>>8;				
				MB_Buf[MB_Buf_size++] = value&0xFF;				
				
				CoilsBuf[start] = value==0xFF00?true:false;
				
				if ( error ){
					MB_Buf_clear();
					MB_Buf[MB_Buf_size++] = addr;
					MB_Buf[MB_Buf_size++] = func+0x80;
					MB_Buf[MB_Buf_size++] = ERR_SERVER_DEVICE_FAILURE;
				}
			}
			crc = CRC16((uint8_t*)MB_Buf, MB_Buf_size);
			MB_Buf[MB_Buf_size++] = crc&0xFF;
			MB_Buf[MB_Buf_size++] = crc>>8;
			demand_of_emmision = true;
			break;
			
		case FUN_READ_DISCRETE_INPUTS: // read discretes input
			start 	 = (MB_Buf[2]<<8)+MB_Buf[3];
			quantity = (MB_Buf[4]<<8)+MB_Buf[5];
		
			MB_Buf_clear(); // clear for next message
			if( !DISCRETES_INPUT_ENABLED){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_FUNCTION;	
			} else if(quantity <= 0 || quantity > 2000){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_VALUE;		
			} else if(start+0 < DISCRETES_INPUT_START || start+quantity > DISCRETES_INPUT_START+DISCRETES_INPUT_NREGS){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_ADDRESS;		
			} else {
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				
				MB_Buf[MB_Buf_size++] = quantity/8+((quantity%8)==0?0:1);				
				for(i=0; i<quantity;){
					MB_Buf[MB_Buf_size] += (DiscretesInputBuf[start+i]?1:0)<<(i%8);
					++i;
					if(i%8==0 && i!=0) ++MB_Buf_size;
				}
				if(quantity%8 != 0) ++MB_Buf_size;	
				
				if ( error ){
					MB_Buf_clear();
					MB_Buf[MB_Buf_size++] = addr;
					MB_Buf[MB_Buf_size++] = func+0x80;
					MB_Buf[MB_Buf_size++] = ERR_SERVER_DEVICE_FAILURE;
				}
			}
			crc = CRC16((uint8_t*)MB_Buf, MB_Buf_size);
			MB_Buf[MB_Buf_size++] = crc&0xFF;
			MB_Buf[MB_Buf_size++] = crc>>8;
			demand_of_emmision = true;
			break;		

			
		case FUN_READ_INPUT_REGISTER: // read input register
			start 	 = (MB_Buf[2]<<8)+MB_Buf[3];
			quantity = (MB_Buf[4]<<8)+MB_Buf[5];
		
			MB_Buf_clear(); // clear for next message
			if( !INPUT_REGISTERS_ENABLED ){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_FUNCTION;	
			} else if(/*value < 0x0000 ||*/ value > 0xFFFF){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_VALUE;		
			} else if(start+0 < INPUT_REGISTERS_START || start+quantity > INPUT_REGISTERS_START+INPUT_REGISTERS_NREGS){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_ADDRESS;		
			} else {
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = quantity*2;
				for(i = 0; i < quantity; ++i){
					MB_Buf[MB_Buf_size++] = InputRegistersBuf[start+i]>>8;				
					MB_Buf[MB_Buf_size++] = InputRegistersBuf[start+i]&0xFF;		
				}
				
				if ( error ){
					MB_Buf_clear();
					MB_Buf[MB_Buf_size++] = addr;
					MB_Buf[MB_Buf_size++] = func+0x80;
					MB_Buf[MB_Buf_size++] = ERR_SERVER_DEVICE_FAILURE;
				}
			}
			crc = CRC16((uint8_t*)MB_Buf, MB_Buf_size);
			MB_Buf[MB_Buf_size++] = crc&0xFF;
			MB_Buf[MB_Buf_size++] = crc>>8;
			demand_of_emmision = true;
			break;			
			
		case FUN_READ_HOLDING_REGISTER: // read holding register
			start 	 = (MB_Buf[2]<<8)+MB_Buf[3];
			quantity = (MB_Buf[4]<<8)+MB_Buf[5];
		
			MB_Buf_clear(); // clear for next message
			if( !HOLDING_REGISTERS_ENABLED ){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_FUNCTION;	
			} else if(/*value < 0x0000 ||*/ value > 0xFFFF){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_VALUE;		
			} else if(start+0 < HOLDING_REGISTERS_START || start+quantity > HOLDING_REGISTERS_START+HOLDING_REGISTERS_NREGS){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_ADDRESS;		
			} else {
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = quantity*2;
				for(i = 0; i < quantity; ++i){
					MB_Buf[MB_Buf_size++] = HoldingRegistersBuf[start+i]>>8;				
					MB_Buf[MB_Buf_size++] = HoldingRegistersBuf[start+i]&0xFF;		
				}
				
				if ( error ){
					MB_Buf_clear();
					MB_Buf[MB_Buf_size++] = addr;
					MB_Buf[MB_Buf_size++] = func+0x80;
					MB_Buf[MB_Buf_size++] = ERR_SERVER_DEVICE_FAILURE;
				}
			}
			crc = CRC16((uint8_t*)MB_Buf, MB_Buf_size);
			MB_Buf[MB_Buf_size++] = crc&0xFF;
			MB_Buf[MB_Buf_size++] = crc>>8;
			demand_of_emmision = true;
			break;			
			
		case FUN_WRITE_SINGLE_REGISTER: // write holding register
			start 	 = (MB_Buf[2]<<8)+MB_Buf[3];
			value    = (MB_Buf[4]<<8)+MB_Buf[5];
		
			MB_Buf_clear(); // clear for next message
			if( !INPUT_REGISTERS_ENABLED ){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_FUNCTION;	
			} else if(/*value < 0x0000 ||*/ value > 0xFFFF){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_VALUE;		
			} else if(start+0 < INPUT_REGISTERS_START || start >= INPUT_REGISTERS_START+INPUT_REGISTERS_NREGS){
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func+0x80;
				MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_DATA_ADDRESS;		
			} else {
				MB_Buf[MB_Buf_size++] = addr;
				MB_Buf[MB_Buf_size++] = func;
				MB_Buf[MB_Buf_size++] = start>>8;				
				MB_Buf[MB_Buf_size++] = start&0xFF;				
				MB_Buf[MB_Buf_size++] = value>>8;				
				MB_Buf[MB_Buf_size++] = value&0xFF;				
				
				HoldingRegistersBuf[start] = value;
				
				if ( error ){
					MB_Buf_clear();
					MB_Buf[MB_Buf_size++] = addr;
					MB_Buf[MB_Buf_size++] = func+0x80;
					MB_Buf[MB_Buf_size++] = ERR_SERVER_DEVICE_FAILURE;
				}
			}
			crc = CRC16((uint8_t*)MB_Buf, MB_Buf_size);
			MB_Buf[MB_Buf_size++] = crc&0xFF;
			MB_Buf[MB_Buf_size++] = crc>>8;
			demand_of_emmision = true;
			break;			
			
		default:
			MB_Buf_clear();
			MB_Buf[MB_Buf_size++] = addr;
			MB_Buf[MB_Buf_size++] = func+0x80;
			MB_Buf[MB_Buf_size++] = ERR_ILLEGAL_FUNCTION;	
			crc = CRC16((uint8_t*)MB_Buf, MB_Buf_size);
			MB_Buf[MB_Buf_size++] = crc&0xFF;
			MB_Buf[MB_Buf_size++] = crc>>8;
			demand_of_emmision = true;
			break;
	}
}

void MB_Buf_clear(void){
	int i = 0;
	for(i=0; i<MB_Buf_size; ++i){
		MB_Buf[i] = 0;
	}
	MB_Buf_size = 0;
}

// temporary implementation -- so that it compiles!
__weak void Communication_Put(uint8_t c){} 
__weak uint8_t Communication_Get(void){return 0;}
__weak void Communication_Mode(bool rx, bool tx){}
__weak void Enable50usTimer(void){}
__weak void Disable50usTimer(void){}
