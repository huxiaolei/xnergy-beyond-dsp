/*
 * modbus-rtu.c
 *
 *  Created on: Apr 30, 2017
 *      Author: xlhu
 */

#include <stdint.h>
#include <stdbool.h>
//#include <stdlib.h>
//#include <stdarg.h>
//#include <errno.h>
//#include <limits.h>
#ifdef TEST
#include <assert.h>
#endif
#include "hal_defs.h"
#include "modbus-rtu.h"
#include "bsp_uart.h"
#include "bsp_time.h"

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03,
		0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C,
		0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
		0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17,
		0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30,
		0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35,
		0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B,
		0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24,
		0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21,
		0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6,
		0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8,
		0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD,
		0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2,
		0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53,
		0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59,
		0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E,
		0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47,
		0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

#ifdef TEST
uint16_t crc16(uint8_t *buffer, uint16_t buffer_length) ;
int16_t _modbus_rtu_check_integrity(uint8_t *msg, int16_t msg_length);

void _respondIoStatus(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bits, int16_t start_address, uint8_t * pdata);
void _respondRegStatus(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bytes, int16_t start_address, uint16_t * pregs);
void _respondWriteMultiCoils(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bits, int16_t start_address) ;
//static void _respondWriteMultiRegs(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bytes, int16_t start_address) ;
void _respondEcho(modbus_backend_t * self);
void _respondException(modbus_backend_t * self, uint8_t slave_id, uint8_t function, uint8_t exception_code) ;
void _errorHandling(modbus_backend_t * self) ;

modbus_rtu_receiver_t * InitModbusReceive(uart_handler_t * puart) ;
modbus_rtu_transmitter_t * initModbusTransmit(uart_handler_t * puart) ;
NON_BLOCKING_RTN modbusRtuReceive(void);
NON_BLOCKING_RTN modbusRtuTransmit(void);

bool _isNewByte(modbus_rtu_receiver_t * self) ;
void _flush(modbus_rtu_receiver_t * self);
bool _receiveWaitFinish(modbus_rtu_receiver_t * self);
bool _shortWaitFinish(modbus_rtu_receiver_t * self);
bool _longWaitFinish(modbus_rtu_receiver_t * self);

bool _isTransmitTimeout(modbus_rtu_transmitter_t * self) ;
bool _isTransmitWaitFinish(modbus_rtu_transmitter_t * self) ;
void _appendCrc16(modbus_rtu_transmitter_t * pmodbusRtuTransmitter) ;
void _buildRespondHeader(uint8_t slave_id, uint8_t function) ;
#else
static uint16_t crc16(uint8_t *buffer, uint16_t buffer_length) ;
static int16_t _modbus_rtu_check_integrity(uint8_t *msg, int16_t msg_length);

static void _respondIoStatus(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bits, int16_t start_address, uint8_t * pdata);
static void _respondRegStatus(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bytes, int16_t start_address, uint16_t * pregs);
static void _respondWriteMultiCoils(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bits, int16_t start_address) ;
//static void _respondWriteMultiRegs(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bytes, int16_t start_address) ;
static void _respondEcho(modbus_backend_t * self);
static void _respondException(modbus_backend_t * self, uint8_t slave_id, uint8_t function, uint8_t exception_code) ;
static void _errorHandling(modbus_backend_t * self) ;

static modbus_rtu_receiver_t * InitModbusReceive(uart_handler_t * puart) ;
static modbus_rtu_transmitter_t * initModbusTransmit(uart_handler_t * puart) ;
static NON_BLOCKING_RTN modbusRtuReceive(void);
static NON_BLOCKING_RTN modbusRtuTransmit(void);

static bool _isNewByte(modbus_rtu_receiver_t * self) ;
static void _flush(modbus_rtu_receiver_t * self);
static bool _receiveWaitFinish(modbus_rtu_receiver_t * self);
static bool _shortWaitFinish(modbus_rtu_receiver_t * self);
static bool _longWaitFinish(modbus_rtu_receiver_t * self);

static bool _isTransmitTimeout(modbus_rtu_transmitter_t * self) ;
static bool _isTransmitWaitFinish(modbus_rtu_transmitter_t * self) ;
static void _appendCrc16(modbus_rtu_transmitter_t * pmodbusRtuTransmitter) ;
static void _buildRespondHeader(uint8_t slave_id, uint8_t function) ;
#endif

uint8_t modbus_receive_buffer[MODBUS_RTU_MAX_ADU_LENGTH];
uint8_t modbus_transmit_buffer[MODBUS_RTU_MAX_ADU_LENGTH];

modbus_rtu_receiver_t modbusRtuReceiver ;
modbus_rtu_transmitter_t modbusRtuTransmitter;
modbus_rtu_receiver_t * pmodbusRtuReceiver ;
modbus_rtu_transmitter_t * pmodbusRtuTransmitter;

//implement modbus_backend_t
modbus_backend_t modbus_backend_rtu ;

modbus_backend_t *
requestModbusBackendRtu(uint32_t baudRate){
	//init functions
	uart_handler_t * puartHandler ;
	puartHandler = initUart(baudRate) ;
	//open uart
	puartHandler->openUart0(puartHandler,baudRate);

	pmodbusRtuReceiver = InitModbusReceive(puartHandler);
	pmodbusRtuTransmitter = initModbusTransmit(puartHandler);
	//
	modbus_backend_rtu.rxBuffer = pmodbusRtuReceiver->buffer ;

	//
	// assign funcs
	//
	modbus_backend_rtu.receiveMsg =  modbusRtuReceive ;
	modbus_backend_rtu.sendMsg = modbusRtuTransmit ;
	modbus_backend_rtu.respondIoStatus = _respondIoStatus ;
	modbus_backend_rtu.respondRegStatus = _respondRegStatus ;
	modbus_backend_rtu.respondWriteMultiCoils = _respondWriteMultiCoils ;
//	modbus_backend_rtu.respondWriteMultiRegs = _respondWriteMultiRegs ; _respondWriteMultiCoils
	modbus_backend_rtu.respondWriteMultiRegs = _respondWriteMultiCoils ;
	modbus_backend_rtu.respondEcho = _respondEcho ;
	modbus_backend_rtu.respondException = _respondException ;
	modbus_backend_rtu.errorHandling = _errorHandling ;

	return &modbus_backend_rtu;
}

#ifdef TEST
void
#else
static void
#endif
_buildRespondHeader(uint8_t slave_id, uint8_t function){
	pmodbusRtuTransmitter->tx_length = 0 ;
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = slave_id ;
	pmodbusRtuTransmitter->tx_length++ ;
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = function;
	pmodbusRtuTransmitter->tx_length++ ;
}

#ifdef TEST
void
#else
static void
#endif
_respondIoStatus(modbus_backend_t * self, uint8_t slave_id, uint8_t function,
		int16_t num_bits, int16_t start_address, uint8_t * pdata){

	int16_t return_bytes;
	return_bytes = num_bits/8 +((num_bits%8)?1:0) ;
	//build header
	_buildRespondHeader(slave_id, function);
	//bytes count
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = return_bytes;
	pmodbusRtuTransmitter->tx_length++ ;
	//data
    uint8_t shift = 0;
    /* Instead of byte (not allowed in Win32) */
    uint8_t one_byte = 0;
    int16_t i;

    for (i = start_address; i < start_address + num_bits; i++) {
        one_byte |= pdata[i] << shift;
        if (shift == 7) {
            /* Byte is full */
        	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = one_byte;
        	pmodbusRtuTransmitter->tx_length++ ;
            one_byte = shift = 0;
        } else {
            shift++;
        }
    }

    if (shift != 0){
    	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = one_byte;
    	pmodbusRtuTransmitter->tx_length++ ;
    }

    _appendCrc16(pmodbusRtuTransmitter);

}

#ifdef TEST
void
#else
static void
#endif
_respondRegStatus(modbus_backend_t * self, uint8_t slave_id, uint8_t function,
		int16_t num_regs, int16_t start_address, uint16_t * pregs){

	//header
	_buildRespondHeader(slave_id, function);
	//bytes count
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = num_regs*2;
	pmodbusRtuTransmitter->tx_length++ ;
	//bytes
	int16_t i;
	 for (i = start_address; i < start_address + num_regs; i++){
		pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = HI_UINT16(pregs[i]);
		pmodbusRtuTransmitter->tx_length++ ;
		pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = LO_UINT16(pregs[i]);
		pmodbusRtuTransmitter->tx_length++ ;
	 }

	 //crc
	 _appendCrc16(pmodbusRtuTransmitter);

}

#ifdef TEST
void
#else
static void
#endif
_respondWriteMultiCoils(modbus_backend_t * self, uint8_t slave_id, uint8_t function,
		int16_t num_bits, int16_t start_address){
	//header
	_buildRespondHeader(slave_id, function);
	//coil start address
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = HI_UINT16(start_address);
	pmodbusRtuTransmitter->tx_length++ ;
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = LO_UINT16(start_address);
	pmodbusRtuTransmitter->tx_length++ ;
	//num of bits
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = HI_UINT16(num_bits);
	pmodbusRtuTransmitter->tx_length++ ;
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = LO_UINT16(num_bits);
	pmodbusRtuTransmitter->tx_length++ ;
	//crc
	_appendCrc16(pmodbusRtuTransmitter);
}

#ifdef TEST
void
#else
static void
#endif
_respondEcho(modbus_backend_t * self){
	//copy from tx to rx
	int16_t i ;
	for(i=0; i<pmodbusRtuReceiver->rx_length ;i++){
		//
		pmodbusRtuTransmitter->buffer[i] = pmodbusRtuReceiver->buffer[i];
	}
	pmodbusRtuTransmitter->tx_length = pmodbusRtuReceiver->rx_length ;
}

#ifdef TEST
void
#else
static void
#endif
_respondException(modbus_backend_t * self, uint8_t slave_id, uint8_t function,
		uint8_t exception_code){
	//header
	_buildRespondHeader(slave_id, function+0x80) ;
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = exception_code;
	pmodbusRtuTransmitter->tx_length++ ;
	_appendCrc16(pmodbusRtuTransmitter);
}

#ifdef TEST
void
#else
static void
#endif
_errorHandling(modbus_backend_t * self){

}


#ifdef TEST
modbus_rtu_receiver_t *
#else
static modbus_rtu_receiver_t *
#endif
InitModbusReceive(uart_handler_t * puart){
	modbus_rtu_receiver_t * self ;
	self = &modbusRtuReceiver ;
	self->buffer = &modbus_receive_buffer[0] ;
	self->rx_length = 0 ;
	self->puart = puart ;
	//
	//get system current time
	//
	self->time = systemTime.getTimeuSec() ;
	//
	//configure cfgtime
	//
	self->cfg_wait_time = MODBUS_CHAR_TIME_BASE(self->puart->baudRate)*MODBUS_RTU_RX_WAIT_HALF_CHAR;
	self->cfg_short_wait_time = MODBUS_CHAR_TIME_BASE(self->puart->baudRate)*MODBUS_RTU_RX_SHORT_WAIT_HALF_CHAR;
	self->cfg_long_wait_time = MODBUS_CHAR_TIME_BASE(self->puart->baudRate)*MODBUS_RTU_RX_LONG_WAIT_HALF_CHAR;
	//function
	self->isNewByte = _isNewByte;
	self->flush = _flush ;
	self->receiveWaitFinish = _receiveWaitFinish ;
	self->shortWaitFinish = _shortWaitFinish ;
	self->longWaitFinish = _longWaitFinish ;
	//
	self->current_status = MODBUS_RTU_RECEIVER_SM_LONG_WAIT ;
	self->previous_status = MODBUS_RTU_RECEIVER_SM_LONG_WAIT ;
	return self ;
}

// this is a non blocking call
#ifdef TEST
NON_BLOCKING_RTN
#else
static NON_BLOCKING_RTN
#endif
modbusRtuReceive(void){
	int32_t rs;
	rs = PENDING ;
	switch(pmodbusRtuReceiver->current_status){
	case MODBUS_RTU_RECEIVER_SM_LONG_WAIT:
		// wait for 3.5 chars
		if(pmodbusRtuReceiver->isNewByte(pmodbusRtuReceiver)){
			//flush new byte
			pmodbusRtuReceiver->flush(pmodbusRtuReceiver);
			pmodbusRtuReceiver->time = systemTime.getTimeuSec() ;
			rs = PENDING ;
		}
		else{
			if(pmodbusRtuReceiver->longWaitFinish(pmodbusRtuReceiver)){
				//jump to ready, and clean up
				pmodbusRtuReceiver->current_status = MODBUS_RTU_RECEIVER_SM_READY ;
				pmodbusRtuReceiver->previous_status = MODBUS_RTU_RECEIVER_SM_LONG_WAIT ;
				pmodbusRtuReceiver->flush(pmodbusRtuReceiver);
				//rs = PENDING ;
			}
			//else{
			rs = PENDING;
			//}
		}
		break;
	case MODBUS_RTU_RECEIVER_SM_READY:
		//wait for new byte to come
		if(pmodbusRtuReceiver->isNewByte(pmodbusRtuReceiver)){
			pmodbusRtuReceiver->current_status = MODBUS_RTU_RECEIVER_SM_RECEIVE ;
			pmodbusRtuReceiver->previous_status = MODBUS_RTU_RECEIVER_SM_READY ;
			//self->enableDelayTimer(self);
			//rest buffer
			pmodbusRtuReceiver->rx_length = 0 ;
			pmodbusRtuReceiver->time = systemTime.getTimeuSec() ;
			//rs = SUCCESS ;
		}
		//else{
			//stay in ready state and wait for new byte come in
		rs = PENDING ;
		//}
		break;
	case MODBUS_RTU_RECEIVER_SM_RECEIVE:
		//check new byte come in
		if(pmodbusRtuReceiver->isNewByte(pmodbusRtuReceiver)){
			//receive new byte
			while(pmodbusRtuReceiver->isNewByte(pmodbusRtuReceiver)){
				pmodbusRtuReceiver->buffer[pmodbusRtuReceiver->rx_length] = UARTCharGetNonBlocking(pmodbusRtuReceiver->puart->uartBase) ;
				pmodbusRtuReceiver->rx_length++;
				//reset timer
				pmodbusRtuReceiver->time = systemTime.getTimeuSec() ;
			}
			rs = PENDING ;
		}
		else{
			if(pmodbusRtuReceiver->receiveWaitFinish(pmodbusRtuReceiver)){
				pmodbusRtuReceiver->current_status = MODBUS_RTU_RECEIVER_SM_SHORT_WAIT ;
				pmodbusRtuReceiver->previous_status = MODBUS_RTU_RECEIVER_SM_RECEIVE ;
				//rs = SUCCESS ;
			}
			//else{
			rs = PENDING ;
			//}
		}
		break;
	case MODBUS_RTU_RECEIVER_SM_SHORT_WAIT:
		//check new byte come in?
		if(pmodbusRtuReceiver->isNewByte(pmodbusRtuReceiver)){
			//flush received byte
			pmodbusRtuReceiver->flush(pmodbusRtuReceiver);
			pmodbusRtuReceiver->current_status = MODBUS_RTU_RECEIVER_SM_LONG_WAIT ;
			pmodbusRtuReceiver->previous_status = MODBUS_RTU_RECEIVER_SM_SHORT_WAIT ;
			//self->resetDelayTimer(self);
			pmodbusRtuReceiver->time = systemTime.getTimeuSec() ;
			modbus_backend_rtu.error_code = ERROR_MODBUS_BACKEND_RECEIVE_TIMEOUT | modbus_backend_rtu.error_code ;
			rs = FAILED ;
		}
		else{
			if(pmodbusRtuReceiver->shortWaitFinish(pmodbusRtuReceiver)){
				pmodbusRtuReceiver->current_status = MODBUS_RTU_RECEIVER_SM_FRAME_RECEIVED ;
				pmodbusRtuReceiver->previous_status = MODBUS_RTU_RECEIVER_SM_SHORT_WAIT ;
				//self->disableDelayTimer(self);
				//self->resetDelayTimer(self);
				pmodbusRtuReceiver->time = systemTime.getTimeuSec() ;
				//rs = SUCCESS ;
			}
			//else{
			rs = PENDING ;
			//}
		}
		break;
	case MODBUS_RTU_RECEIVER_SM_FRAME_RECEIVED:
		//invoke app function
		//pmodbusRtuReceiver->receiveAppFunc(pmodbusRtuReceiver->buffer);
		//jump to ready, when finish.
		pmodbusRtuReceiver->current_status = MODBUS_RTU_RECEIVER_SM_READY ;
		pmodbusRtuReceiver->previous_status = MODBUS_RTU_RECEIVER_SM_FRAME_RECEIVED ;
		//mini length is 2
		rs = FAILED ;
		if(pmodbusRtuReceiver->rx_length>2){
		//crc16 check
			if(_modbus_rtu_check_integrity(pmodbusRtuReceiver->buffer, pmodbusRtuReceiver->rx_length) == pmodbusRtuReceiver->rx_length){
				rs = SUCCESS ;
			}
			else{
				//todo: crc error
				modbus_backend_rtu.error_code = ERROR_MODBUS_BACKEND_INTEGRITY_FAIL | modbus_backend_rtu.error_code ;
			}
		}
		else{
			//todo: received length to short.
			modbus_backend_rtu.error_code = ERROR_MODBUS_BACKEND_SHORT_FRAME | modbus_backend_rtu.error_code ;
		}
		break;
	}
	return rs ;
};

#ifdef TEST
bool
#else
static bool
#endif
_isNewByte(modbus_rtu_receiver_t * self){
	// check uart
	return UARTCharsAvail(self->puart->uartBase);
}

#ifdef TEST
void
#else
static void
#endif
_flush(modbus_rtu_receiver_t * self){
	//empty rx fifo
	while(UARTCharsAvail(self->puart->uartBase)){
		if(UARTCharGetNonBlocking(self->puart->uartBase) == -1){
			break;
		}
	}
}

#ifdef TEST
bool
#else
static bool
#endif
_receiveWaitFinish(modbus_rtu_receiver_t * self){
	return systemTime.getTimeuSec() - self->time >= self->cfg_wait_time;
}

#ifdef TEST
bool
#else
static bool
#endif
_shortWaitFinish(modbus_rtu_receiver_t * self){
	return systemTime.getTimeuSec() - self->time >= self->cfg_short_wait_time;
}

#ifdef TEST
bool
#else
static bool
#endif
_longWaitFinish(modbus_rtu_receiver_t * self){
	return systemTime.getTimeuSec() - self->time >= self->cfg_long_wait_time;
}

#ifdef TEST
modbus_rtu_transmitter_t *
#else
static modbus_rtu_transmitter_t *
#endif
initModbusTransmit(uart_handler_t * puart){
	modbus_rtu_transmitter_t * self;
	self = &modbusRtuTransmitter ;
	self->puart = puart ;
	self->buffer = &modbus_transmit_buffer[0];
	self->isTimeout = _isTransmitTimeout ;
	self->isWaitFinish = _isTransmitWaitFinish ;
	self->time = systemTime.getTimeuSec() ;
	self->cfg_timeout = MODBUS_CHAR_TIME_BASE(self->puart->baudRate)*MODBUS_RTU_TX_TIMEOUT_HALF_CHAR ;
	self->cfg_wait_time = MODBUS_CHAR_TIME_BASE(self->puart->baudRate)*MODBUS_RTU_TX_FINISH_WAIT_HALF_CHAR ;
	//init state
	self->current_status = MODBUS_RTU_TRANSMITTER_SM_READY ;
	self->previous_status = MODBUS_RTU_TRANSMITTER_SM_READY ;
	return self;
}

//
//
// this is a non blocking call
#ifdef TEST
NON_BLOCKING_RTN
#else
static NON_BLOCKING_RTN
#endif
modbusRtuTransmit(void){
	switch(pmodbusRtuTransmitter->current_status){
	case MODBUS_RTU_TRANSMITTER_SM_READY:
		pmodbusRtuTransmitter->current_status = MODBUS_RTU_TRANSMITTER_SM_TRANSMIT ;
		pmodbusRtuTransmitter->previous_status = MODBUS_RTU_TRANSMITTER_SM_READY ;
		//enable transmit by using dma
		pmodbusRtuTransmitter->puart->uart0SendMsgDma(pmodbusRtuTransmitter->tx_length, pmodbusRtuTransmitter->buffer);
		//reset time to current time;
		pmodbusRtuTransmitter->time = systemTime.getTimeuSec() ;
		return PENDING;
	case MODBUS_RTU_TRANSMITTER_SM_TRANSMIT:
		if(pmodbusRtuTransmitter->isTimeout(pmodbusRtuTransmitter)){
			pmodbusRtuTransmitter->current_status = MODBUS_RTU_TRANSMITTER_SM_READY ;
			pmodbusRtuTransmitter->previous_status = MODBUS_RTU_TRANSMITTER_SM_TRANSMIT ;
			//pmodbusRtuTransmitter->error = ERROR_MODBUS_RTU_TX_TIMEOUT ;
			pmodbusRtuTransmitter->tx_length = 0 ;
			modbus_backend_rtu.error_code = ERROR_MODBUS_BACKEND_TRANSMIT_TIMEOUT | modbus_backend_rtu.error_code ;
			return FAILED;
		}
		else{
			//check transmit finish
			if(pmodbusRtuTransmitter->puart->isUart0TxFinish()){
				pmodbusRtuTransmitter->current_status = MODBUS_RTU_TRANSMITTER_SM_WAIT ;
				pmodbusRtuTransmitter->previous_status = MODBUS_RTU_TRANSMITTER_SM_TRANSMIT ;
				pmodbusRtuTransmitter->time = systemTime.getTimeuSec() ;
				return PENDING;
			}
			else{
				return PENDING;
			}
		}
		//break;
	case MODBUS_RTU_TRANSMITTER_SM_WAIT:
		if(pmodbusRtuTransmitter->isWaitFinish(pmodbusRtuTransmitter)){
			pmodbusRtuTransmitter->current_status = MODBUS_RTU_TRANSMITTER_SM_READY ;
			pmodbusRtuTransmitter->previous_status = MODBUS_RTU_TRANSMITTER_SM_WAIT ;
			pmodbusRtuTransmitter->tx_length = 0 ;
			return SUCCESS;
		}
		else{
			return PENDING;
		}
		//break;
	}

	return FAILED;
}

bool
_isTransmitTimeout(modbus_rtu_transmitter_t * self){
	if((systemTime.getTimeuSec() - self->time ) > self->cfg_timeout)
		return true;
	else
		return false;
}

bool
_isTransmitWaitFinish(modbus_rtu_transmitter_t * self){
	if((systemTime.getTimeuSec() - self->time) > self->cfg_wait_time)
		return true;
	else
		return false;
}

#ifdef TEST
void
#else
static void
#endif
_appendCrc16(modbus_rtu_transmitter_t * pmodbusRtuTransmitter){
	//prepare tx data
	//calculate crc 16;
	uint16_t crc16rs ;
	crc16rs = crc16(pmodbusRtuTransmitter->buffer, pmodbusRtuTransmitter->tx_length) ;
	//append
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = HI_UINT16(crc16rs) ;
	pmodbusRtuTransmitter->tx_length++;
	pmodbusRtuTransmitter->buffer[pmodbusRtuTransmitter->tx_length] = LO_UINT16(crc16rs) ;
	pmodbusRtuTransmitter->tx_length++;
}

/* The check_crc16 function shall return 0 if the message is ignored and the
 message length if the CRC is valid. Otherwise it shall return -1 and set
 error code. */
#ifdef TEST
int16_t
#else
static int16_t
#endif
_modbus_rtu_check_integrity(uint8_t *msg, int16_t msg_length){
	uint16_t crc_calculated;
	uint16_t crc_received;
	//int16_t slave = msg[0];

	crc_calculated = crc16(msg, msg_length - 2);
	crc_received = (msg[msg_length - 2] << 8) | msg[msg_length - 1];

	/* Check CRC of msg */
	if (crc_calculated == crc_received) {
		return msg_length;
	}
	else {
		return -1;
	}
}

#ifdef TEST
uint16_t
#else
static uint16_t
#endif
crc16(uint8_t *buffer, uint16_t buffer_length){
    uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
    uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
    uint8_t i; /* will index into CRC lookup */

    /* pass through message buffer */
    while (buffer_length--) {
        i = crc_hi ^ *buffer++; /* calculate the CRC  */
        crc_hi = crc_lo ^ table_crc_hi[i];
        crc_lo = table_crc_lo[i];
    }

    return (crc_hi << 8 | crc_lo);
}
