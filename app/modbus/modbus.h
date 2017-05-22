/*
 * modbus.h
 *
 * **************usage*******************
 * this is an modbus application based on non blocking call method and relies an infinite loop to call the core function.
 * 1. modbus_mapping_t must be declared in main application. this is a mapping of the virtual modbus address
 * to the actual physical mcu address
 * 2. initModbusServer(uint32_t baudrate) must be called to init modbus server.
 * there are few things this function do. it request an backend hardware to support, at this moment, only rtu
 * through uart is supported.
 * baudrate is the actul baudrate, bits size is 8, non parity and one stop bit.
 * 3. RTU mode requries DMA supporting. so the dma must be initialized before using modbus
 * 4. call modbus_server.modbusServerApp(&modbus_server, &modbus_mapping); in a loop.
 * 5. user can hookup the receiveHandler or errorHandler
 * receiveHandler is called when a new valid frame is been received
 * errorHandler is called when an error frame been received.
 *
 * example code:
 *   InitSysTickInt(SYS_TICKS_PEROID, &appTimerIsr);
 *   initDma() ;
 *   initModbusServer(9600);
 *   while(true){
 *    	modbus_server.modbusServerApp(&modbus_server, &modbus_mapping);
 *   };
 *
 *test code can referred to test_modbus.c under root folder.
 *  Created on: May 2, 2017
 *      Author: xlhu
 */

#ifndef APP_MODBUS_MODBUS_H_
#define APP_MODBUS_MODBUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "hal_defs.h"

#define DEFAULT_SLAVE_ID 0x10


// error code
#define ERROR_MODBUS_BASE						0x00000000
#define ERROR_MODUBS_NO_BACKEND					(ERROR_MODBUS_BASE|0x0001)
#define ERROR_MODBUS_BACKEND_BASE				0x00010000
#define ERROR_MODBUS_BACKEND_INTEGRITY_FAIL		(ERROR_MODBUS_BACKEND_BASE|0x0001)
#define ERROR_MODBUS_BACKEND_SHORT_FRAME		(ERROR_MODBUS_BACKEND_BASE|0x0002)
#define ERROR_MODBUS_BACKEND_RECEIVE_TIMEOUT	(ERROR_MODBUS_BACKEND_BASE|0x0004)
#define ERROR_MODBUS_BACKEND_TRANSMIT_TIMEOUT	(ERROR_MODBUS_BACKEND_BASE|0x0008)

/* Modbus function codes */
#define MODBUS_FC_READ_COILS                0x01
#define MODBUS_FC_READ_DISCRETE_INPUTS      0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_READ_INPUT_REGISTERS      0x04
#define MODBUS_FC_WRITE_SINGLE_COIL         0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_READ_EXCEPTION_STATUS     0x07
#define MODBUS_FC_WRITE_MULTIPLE_COILS      0x0F
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10
//#define MODBUS_FC_REPORT_SLAVE_ID           0x11
//#define MODBUS_FC_MASK_WRITE_REGISTER       0x16
//#define MODBUS_FC_WRITE_AND_READ_REGISTERS  0x17

#define MODBUS_BROADCAST_ADDRESS    0

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 1 page 12)
 * Quantity of Coils to read (2 bytes): 1 to 2000 (0x7D0)
 * (chapter 6 section 11 page 29)
 * Quantity of Coils to write (2 bytes): 1 to 1968 (0x7B0)
 */
#define MODBUS_MAX_READ_BITS              2000
#define MODBUS_MAX_WRITE_BITS             1968

/* Modbus_Application_Protocol_V1_1b.pdf (chapter 6 section 3 page 15)
 * Quantity of Registers to read (2 bytes): 1 to 125 (0x7D)
 * (chapter 6 section 12 page 31)
 * Quantity of Registers to write (2 bytes) 1 to 123 (0x7B)
 * (chapter 6 section 17 page 38)
 * Quantity of Registers to write in R/W registers (2 bytes) 1 to 121 (0x79)
 */
#define MODBUS_MAX_READ_REGISTERS          125
#define MODBUS_MAX_WRITE_REGISTERS         123
#define MODBUS_MAX_WR_WRITE_REGISTERS      121
#define MODBUS_MAX_WR_READ_REGISTERS       125

/* The size of the MODBUS PDU is limited by the size constraint inherited from
 * the first MODBUS implementation on Serial Line network (max. RS485 ADU = 256
 * bytes). Therefore, MODBUS PDU for serial line communication = 256 - Server
 * address (1 byte) - CRC (2 bytes) = 253 bytes.
 */
#define MODBUS_MAX_PDU_LENGTH              253

/* Consequently:
 * - RTU MODBUS ADU = 253 bytes + Server address (1 byte) + CRC (2 bytes) = 256
 *   bytes.
 * - TCP MODBUS ADU = 253 bytes + MBAP (7 bytes) = 260 bytes.
 * so the maximum of both backend in 260 bytes. This size can used to allocate
 * an array of bytes to store responses and it will be compatible with the two
 * backends.
 */
#define MODBUS_MAX_ADU_LENGTH              260

/* Random number to avoid errno conflicts */
//#define MODBUS_ENOBASE 112345678

/* Protocol exceptions */
enum {
    MODBUS_EXCEPTION_ILLEGAL_FUNCTION = 0x01,
    MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS,
    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE,
    MODBUS_EXCEPTION_ACKNOWLEDGE,
    MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY,
    MODBUS_EXCEPTION_NEGATIVE_ACKNOWLEDGE,
    MODBUS_EXCEPTION_MEMORY_PARITY,
    MODBUS_EXCEPTION_NOT_DEFINED,
    MODBUS_EXCEPTION_GATEWAY_PATH,
    MODBUS_EXCEPTION_GATEWAY_TARGET,
    MODBUS_EXCEPTION_MAX
};

typedef void (*ASP_MODBUS_SERVER)(void);

typedef struct _modbus_server_t modbus_server_t ;
typedef struct _modbus_backend_t modbus_backend_t ;

#define MODBUS_COILS_START_ADD				0x00000000
#define MODBUS_DISCRETE_INPUTS_START_ADD 	0x00000000
#define MODBUS_INPUT_REGISTERS_START_ADD	0x00000000
#define MODBUS_HOLDING_REGISTERS_START_ADD	0x00000000

/**
* @brief modbus memory mapping. User needs to initialize this mapping in their application
*
* memory mapping of modbus address space to applition memory address space.
* MODBUS_COILS_START_ADD defines coils start address in modbus address space.
* tab_bits is the physical memory in mcu and tab_bits[0] will be mapped to MODBUS_COILS_START_ADD
* MODBUS_DISCRETE_INPUTS_START_ADD defines discrete input start address
* tab_input_bits is the physical memory in MCU and tab_input_bits[0] is mapped to MODBUS_DISCRETE_INPUTS_START_ADD
* MODBUS_INPUT_REGISTERS_START_ADD defines input regs start address in modbus.
* tab_input_registers is the physical memory in MCU
* MODBUS_HOLDING_REGISTERS_START_ADD defines the start address for holding registers
* tab_registers is the physical memory in MCU.
*/
typedef struct _modbus_mapping_t modbus_mapping_t ;

struct _modbus_mapping_t {

    uint16_t nb_bits;
    //uint8_t * start_bits;	// each bit take one byte space for simplicity
    uint16_t nb_input_bits;
    //uint8_t start_input_bits; // each bit take one byte space for simplicity
    uint16_t nb_input_registers;
    //uint16_t * start_input_registers;
    uint16_t nb_registers;
    //uint16_t * start_registers;

    uint8_t *tab_bits;	//wr
    uint8_t *tab_input_bits; //read only
    uint16_t *tab_input_registers; //read only
    uint16_t *tab_registers; //wr

};

/**
* @brief statistics for modbus communication.
* received_pkts counts all successful packets.
* error_pkts counts all packets with error.
*
*/
typedef struct _modbus_statistics_t {

	uint32_t received_pkts ;
	uint32_t error_pkts ;

}modbus_statistics_t ;

/**
* @brief modbus server.
* modbus server is required to use modbus.
* initModbusServer() returns a modbus server pointer for user to use modbus server.
* pbackend is a backend handler which handles low level protocol. i.e. RTU or TCP.
* pbackend decouples modbus and lwo level implementation.
* pmodbus_statistics contains statistics infomation
* error_code is the error_code when error happens. It will be cleared after errorHandler is invoked.
* To use error_code, error_handler must be implemented in application. other wise it will be cleared.
* slaveId Slave ID, must be configured before use modbus.
* todo: move sendRespond out of modbus server.
* modbusServerApp: handler statemachine related stuff. must be called in the main background while loop
* receiveHandler: will be invoked when receive a frame. ignored if it is not implemented by user.
* errorHandler: will be invoked when error happens.
*/
struct _modbus_server_t{
	modbus_backend_t * pbackend ;
	modbus_statistics_t * pmodbus_statistics ;
	uint32_t error_code ;
	uint8_t slaveId ;
	bool sendRespond ;
	// function
	NON_BLOCKING_RTN (*modbusServerApp)(modbus_server_t * self, modbus_mapping_t * pmapping) ;
	ASP_MODBUS_SERVER receiveHandler ;
	ASP_MODBUS_SERVER errorHandler ;
};


/**
* @brief modbus backend. defines necessary backend variables and functions.
* error_code, error of the backend.
* rxBuffer, a receive buffer
*
*/
struct _modbus_backend_t {
	uint32_t error_code;
	uint8_t * rxBuffer ;

	//function

	//
	// receive a msg
	//
	NON_BLOCKING_RTN (*receiveMsg)(void) ;

	//
	// sending a msg
	//
	NON_BLOCKING_RTN (*sendMsg)(void) ;

	//todo: move all those functions to modbus.c or create a private header file, as
	//those functions are not callable by higher level application.
	//
	// respond to io status, including discrete input and coils.
	//
	void (*respondIoStatus)(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bits, int16_t start_address, uint8_t * pdata);

	//
	// respond to register status including input and holding
	//
	void (*respondRegStatus)(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bytes, int16_t start_address, uint16_t * pregs);

	//
	// respond to multiple coils
	//
	void (*respondWriteMultiCoils)(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bits, int16_t start_address) ;

	//
	// prepare respond to write multiple registers.
	//
	void (*respondWriteMultiRegs)(modbus_backend_t * self, uint8_t slave_id, uint8_t function, int16_t num_bytes, int16_t start_address) ;

	//
	// echo back, for some function code, the server just echo back to acknowledge to the request of the client.
	//
	void (*respondEcho)(modbus_backend_t * self);

	//
	// respond to an exception
	//
	void (*respondException)(modbus_backend_t * self, uint8_t slave_id, uint8_t function, uint8_t exception_code) ;

	//
	// error handling is meant for cleaning up the backend if any error happens.
	//
	void (*errorHandling)(modbus_backend_t * self) ;

};



//backend needs to be initialized before usage
//extern uint32_t modbusServerApp(modbus_backend_t * backend, modbus_mapping_t * mapping) ;

extern modbus_server_t modbus_server ;

/**
* @brief init modbus server,
* @param baudrate: baudrate.
* @return a pinter of modbus_server
*/
extern void initModbusServer(uint32_t baudRate) ;

#endif /* APP_MODBUS_MODBUS_H_ */
