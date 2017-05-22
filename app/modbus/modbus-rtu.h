/*
 * modbus-rtu.h
 *
 *  Created on: Apr 30, 2017
 *      Author: xlhu
 */

#ifndef APP_MODBUS_MODBUS_RTU_H_
#define APP_MODBUS_MODBUS_RTU_H_

#include <stdint.h>
#include <stdbool.h>
#include "hal_defs.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "error.h"
#include "modbus.h"

#define ERROR_MODBUS_RTU_TX (0x00000000)
#define ERROR_MODBUS_RTU_TX_TIMEOUT (ERROR_MODBUS_RTU_BASE |ERROR_MODBUS_RTU_TX | 0x00000001)
#define ERROR_MODBUS_RTU_TX_TIMEOUT_MASK (0x00000001)
#define ERROR_MODBUS_RTU_RX (0X00100000)

/* Modbus_Application_Protocol_V1_1b.pdf Chapter 4 Section 1 Page 5
 * RS232 / RS485 ADU = 253 bytes + slave (1 byte) + CRC (2 bytes) = 256 bytes
 */
#define MODBUS_RTU_MAX_ADU_LENGTH  256
//define baud rate
#define B110	110		//26,624
#define B300 	300		//13,312
#define B600 	600		//6,656
#define B1200 	1200	//3,328
#define B2400 	2400	//1,664
#define B4800 	4800	//832
#define B9600 	9600	//416us
#define B19200 	19200	//208us for half byte
#define B38400 	38400	//104us
//#define B57600 	57600
//#define B115200 115200
#define B110_HALF_CHAR_USEC		26624
#define B300_HALF_CHAR_USEC 	13312
#define B600_HALF_CHAR_USEC 	6656
#define B1200_HALF_CHAR_USEC 	3328
#define B2400_HALF_CHAR_USEC 	1664
#define B4800_HALF_CHAR_USEC 	832
#define B9600_HALF_CHAR_USEC 	416
#define B19200_HALF_CHAR_USEC 	208 //for half byte
#define B38400_HALF_CHAR_USEC 	104

#define MODBUS_RTU_RX_WAIT_HALF_CHAR			3 // 1.5 CHARS
#define MODBUS_RTU_RX_SHORT_WAIT_HALF_CHAR 		4 // 2 CHARS
#define MODBUS_RTU_RX_LONG_WAIT_HALF_CHAR		7 // 3.5 CHARS
#define MODBUS_RTU_TX_FINISH_WAIT_HALF_CHAR		7 // 3.5 CHARS
#define MODBUS_RTU_TX_TIMEOUT_HALF_CHAR			(MODBUS_RTU_MAX_ADU_LENGTH*2)

#define MODBUS_CHAR_TIME_BASE(x) 4*((1000000)/x)
//typedef void (*MODBUS_HOOK_FUNC_PTR)(uint8_t *);

//
// receiver type
//
typedef enum {
	MODBUS_RTU_RECEIVER_SM_LONG_WAIT = 0 ,
	MODBUS_RTU_RECEIVER_SM_READY,
	MODBUS_RTU_RECEIVER_SM_RECEIVE,
	MODBUS_RTU_RECEIVER_SM_SHORT_WAIT,
	MODBUS_RTU_RECEIVER_SM_FRAME_RECEIVED
}modbus_rtu_receiver_sm_t;


typedef struct _modbus_rtu_receiver_t modbus_rtu_receiver_t;

struct _modbus_rtu_receiver_t{
	uart_handler_t * puart;
	modbus_rtu_receiver_sm_t current_status;
	modbus_rtu_receiver_sm_t previous_status;
	uint8_t * buffer;
	uint32_t rx_length ;
//	uint32_t rxPointer ;
	uint32_t time;
	uint32_t cfg_wait_time;
	uint32_t cfg_short_wait_time;
	uint32_t cfg_long_wait_time;

	//====functions
	bool (*isNewByte)(modbus_rtu_receiver_t * self) ;
	void (*resetDelayTimer)(modbus_rtu_receiver_t * self);
	void (*disableDelayTimer)(modbus_rtu_receiver_t * self);
	void (*flush)(modbus_rtu_receiver_t * self);
	void (*enableDelayTimer)(modbus_rtu_receiver_t * self);
	bool (*receiveWaitFinish)(modbus_rtu_receiver_t * self);
	bool (*shortWaitFinish)(modbus_rtu_receiver_t * self);
	bool (*longWaitFinish)(modbus_rtu_receiver_t * self);
	//MODBUS_HOOK_FUNC_PTR receiveAppFunc;

};

//
// transmitter type
//
typedef enum {
	MODBUS_RTU_TRANSMITTER_SM_READY = 0,
	MODBUS_RTU_TRANSMITTER_SM_TRANSMIT,
	MODBUS_RTU_TRANSMITTER_SM_WAIT,
}modbus_rtu_transmitter_sm_t;

typedef struct _modbus_rtu_transmitter_t modbus_rtu_transmitter_t ;

struct _modbus_rtu_transmitter_t {
	uint32_t error;
	uart_handler_t * puart;
	modbus_rtu_transmitter_sm_t current_status;
	modbus_rtu_transmitter_sm_t previous_status;
	uint8_t * buffer;
	uint32_t tx_length ;
	uint32_t time;
	uint32_t cfg_timeout;
	uint32_t cfg_wait_time ;
	//
	//
	//
	bool (*isTimeout)(modbus_rtu_transmitter_t *) ;
	bool (*isWaitFinish)(modbus_rtu_transmitter_t *) ;
};

modbus_backend_t * requestModbusBackendRtu(uint32_t baudRate) ;

#endif /* APP_MODBUS_MODBUS_RTU_H_ */
