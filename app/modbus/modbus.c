/*
 * modbus.c
 *
 *  Created on: May 2, 2017
 *      Author: xlhu
 */


#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#ifdef TEST
#include <assert.h>
#endif
#include "hal_defs.h"
#include "modbus-rtu.h"
#include "modbus.h"

static NON_BLOCKING_RTN _modbusServerApp(modbus_server_t * self, modbus_mapping_t * pmapping) ;
static void _parse_and_reply(modbus_server_t * self, modbus_mapping_t * mb_mapping) ;
void _modbus_set_bits_from_bytes(uint8_t *dest, int16_t idx, uint16_t nb_bits,
                                const uint8_t *tab_byte);
modbus_statistics_t modbus_statistics ;

modbus_server_t modbus_server = {
	//&modbus_server,
	NULL,
	&modbus_statistics,
	0,
	DEFAULT_SLAVE_ID,
	false,
	_modbusServerApp,
	NULL,
	NULL
};


void
initModbusServer(uint32_t baudRate){
	modbus_server.pbackend = requestModbusBackendRtu(baudRate);
}

static NON_BLOCKING_RTN _modbusServerApp(modbus_server_t * self, modbus_mapping_t * pmapping){
	NON_BLOCKING_RTN receive ;
	//first check existing of backend
	if(self->pbackend == NULL){
		//todo: set error code
		self->error_code = ERROR_MODUBS_NO_BACKEND ;
		return FAILED ;
	}

	//check receive
	receive = self->pbackend->receiveMsg() ;

	if(receive == SUCCESS){
		self->pmodbus_statistics->received_pkts++ ;
		// a packet has been received.
		// parse and reply
		_parse_and_reply(self, pmapping);
		if(self->receiveHandler != NULL){
			self->receiveHandler() ;
		}
		return SUCCESS ;
	}
	else if(receive == FAILED){
		self->pmodbus_statistics->error_pkts++;
		self->error_code = self->error_code|self->pbackend->error_code ;
		self->pbackend->error_code = 0 ;
		// handle error
		if(self->errorHandler != NULL){
			//invoke errorhandler
			self->errorHandler();
		}
		return FAILED ;
	}


	//check transmit
	if(self->sendRespond){
		//self->sendRespond = false;
		NON_BLOCKING_RTN sendMsg = self->pbackend->sendMsg();
		if(sendMsg == SUCCESS){
			self->sendRespond = false;
			return SUCCESS ;
		}
		else if( sendMsg == FAILED ){
			//handle sendMsg error
			self->sendRespond = false;
			self->error_code = self->error_code|self->pbackend->error_code ;
			self->pbackend->error_code = 0 ;
			return FAILED ;
		}
	}

	return PENDING;
}

static void _parse_and_reply(modbus_server_t * self, modbus_mapping_t * mb_mapping){
	//parse received packet
	uint8_t slaveId ;
	uint8_t function ;
	//handy pointer for rx buffer
	uint8_t * req ;
	//position to function code
	int16_t offset = 1 ;
	//modbus address for operating, it is not the physical address in mcu.
	int16_t address ;
	// handy temperate address for each category.
	uint16_t vitualAdd ;

	req = self->pbackend->rxBuffer ;
	slaveId = req[offset-1] ; //first byte is the address.
	function = req[offset] ; //next byte is the function.
	address = (req[offset + 1] << 8) + req[offset + 2];
	//mapping_address is offset address which is an offset to actual physical address.
	if( self->slaveId == slaveId){

		switch (function) {
			case MODBUS_FC_READ_COILS:
			case MODBUS_FC_READ_DISCRETE_INPUTS: {
				bool is_input = (function == MODBUS_FC_READ_DISCRETE_INPUTS);
				//int32_t start_bits = is_input ? mb_mapping->start_input_bits : mb_mapping->start_bits;
				int32_t nb_bits = is_input ? mb_mapping->nb_input_bits : mb_mapping->nb_bits;
				uint8_t *tab_bits = is_input ? mb_mapping->tab_input_bits : mb_mapping->tab_bits;
				//const uint8_t * const name = is_input ? "read_input_bits" : "read_bits";
				int32_t nb = (req[offset + 3] << 8) + req[offset + 4];
				/* The mapping can be shifted to reduce memory consumption and it
				   doesn't always start at address zero. */
				//uint32_t mapping_address = address - start_bits;
				vitualAdd = is_input? MODBUS_DISCRETE_INPUTS_START_ADD:MODBUS_COILS_START_ADD;
				int16_t mapping_address = address - vitualAdd;

				if (nb < 1 || MODBUS_MAX_READ_BITS < nb) {
					//handle MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE) ;
					self->sendRespond = true;
				}
				else if (mapping_address < 0 || (mapping_address + nb) > nb_bits) {
				//else if ((mapping_address + nb) > nb_bits) {
					//handle MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS) ;
					self->sendRespond = true;
				}
				else {
					//accept, build response.
					// slaveId, function, byteCount, data (lo to hi), crc(lo to hi)
					self->pbackend->respondIoStatus(self->pbackend, self->slaveId, function, nb, mapping_address, tab_bits) ;
					self->sendRespond = true;
				}
			}
				break;
			case MODBUS_FC_READ_HOLDING_REGISTERS:
			case MODBUS_FC_READ_INPUT_REGISTERS: {
				bool is_input = (function == MODBUS_FC_READ_INPUT_REGISTERS);
				//uint32_t start_registers = is_input ? mb_mapping->start_input_registers : mb_mapping->start_registers;
				int16_t nb_registers = is_input ? mb_mapping->nb_input_registers : mb_mapping->nb_registers;
				uint16_t *tab_registers = is_input ? mb_mapping->tab_input_registers : mb_mapping->tab_registers;
				//const char * const name = is_input ? "read_input_registers" : "read_registers";
				int32_t nb = (req[offset + 3] << 8) + req[offset + 4];
				/* The mapping can be shifted to reduce memory consumption and it
				   doesn't always start at address zero. */
				vitualAdd = is_input? MODBUS_INPUT_REGISTERS_START_ADD:MODBUS_HOLDING_REGISTERS_START_ADD;
				int16_t mapping_address = address - vitualAdd;

				if (nb < 1 || MODBUS_MAX_READ_REGISTERS < nb) {
					//MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE) ;
					self->sendRespond = true;
				}
				else if (mapping_address < 0 || (mapping_address + nb) > nb_registers) {
					//MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS) ;
					self->sendRespond = true;
				}
				else {
					//accept, build response.
					self->pbackend->respondRegStatus(self->pbackend, self->slaveId, function, nb, mapping_address, tab_registers);
					self->sendRespond = true ;
				}
			}
				break;
			case MODBUS_FC_WRITE_SINGLE_COIL: {
				int32_t mapping_address = address - MODBUS_COILS_START_ADD;

				if (mapping_address < 0 || mapping_address >= mb_mapping->nb_bits) {
					//MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS) ;
					self->sendRespond = true;
				}
				else {
					uint16_t data = (req[offset + 3] << 8) + req[offset + 4];

					if (data == 0xFF00 || data == 0x0) {
						//update mapping
						//mb_mapping->tab_bits[mapping_address] = data ? ON : OFF;
						//uint8_t writeData = (data == 0xFF00) ;
						//mb_mapping->writeBits(mb_mapping, 1, mapping_address, &writeData) ;
						mb_mapping->tab_bits[mapping_address] = data ? 1 : 0;
						//echo back
						self->pbackend->respondEcho(self->pbackend);
						self->sendRespond = true;
					}
					else {
						//MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE
						self->pbackend->respondException(self->pbackend,
								self->slaveId,
								function,
								MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE) ;
						self->sendRespond = true;
					}
				}
			}
				break;
			case MODBUS_FC_WRITE_SINGLE_REGISTER: {
				int32_t mapping_address = address - MODBUS_HOLDING_REGISTERS_START_ADD;

				if (mapping_address < 0 || mapping_address >= mb_mapping->nb_registers) {
					//MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS) ;
					self->sendRespond = true;
				}
				else {
					//uint16_t data = (req[offset + 3] << 8) + req[offset + 4];
					//mb_mapping->writeRegs(mb_mapping, 1, mapping_address, &req[offset + 3]);
					int16_t data = (req[offset + 3] << 8) + req[offset + 4];

					mb_mapping->tab_registers[mapping_address] = data;
					//echo back
					self->pbackend->respondEcho(self->pbackend);
					self->sendRespond = true;
				}
			}
				break;
			case MODBUS_FC_WRITE_MULTIPLE_COILS: {
				int16_t nb = (req[offset + 3] << 8) + req[offset + 4];
				int16_t mapping_address = address - MODBUS_COILS_START_ADD;

				if (nb < 1 || MODBUS_MAX_WRITE_BITS < nb) {
					/* May be the indication has been truncated on reading because of
					 * invalid address (eg. nb is 0 but the request contains values to
					 * write) so it's necessary to flush. */
					//MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE) ;
					self->sendRespond = true;
				}
				else if (mapping_address < 0 ||
						   (mapping_address + nb) > mb_mapping->nb_bits) {
					//MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS) ;
					self->sendRespond = true;
				}
				else {
					/* 6 = byte count */
					_modbus_set_bits_from_bytes(mb_mapping->tab_bits, mapping_address, nb,
					                                       &req[offset + 6]);
					self->pbackend->respondWriteMultiCoils(self->pbackend, self->slaveId, function, nb, address) ;
					self->sendRespond = true;
				}
			}
				break;
			case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
				uint16_t nb = (req[offset + 3] << 8) + req[offset + 4];
				int16_t mapping_address = address - MODBUS_HOLDING_REGISTERS_START_ADD;
				int16_t data_byte_count = req[offset+5] ;

				if (nb < 1 || MODBUS_MAX_WRITE_REGISTERS < nb|| nb != data_byte_count/2) {
					//MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE) ;
					self->sendRespond = true;
				}
				else if (mapping_address < 0 ||
						   (mapping_address + nb) > mb_mapping->nb_registers ) {
					//MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS
					self->pbackend->respondException(self->pbackend,
							self->slaveId,
							function,
							MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS) ;
					self->sendRespond = true;
				}
				else {
					//mb_mapping->writeRegs(mb_mapping, nb, mapping_address, &req[offset+6]);
					int16_t i, j;
					for (i = mapping_address, j = 6; i < mapping_address + nb; i++, j += 2) {
						/* 6 and 7 = first value */
						mb_mapping->tab_registers[i] =
							(req[offset + j] << 8) + req[offset + j + 1];
					}
					self->pbackend->respondWriteMultiRegs(self->pbackend, self->slaveId, function, nb, address) ;
					self->sendRespond = true;
				}
			}
				break;
			//case MODBUS_FC_REPORT_SLAVE_ID: {}	break;
			//case MODBUS_FC_READ_EXCEPTION_STATUS:	break;
			//case MODBUS_FC_MASK_WRITE_REGISTER: {}break;
			//case MODBUS_FC_WRITE_AND_READ_REGISTERS: {}break;
			default:
				//MODBUS_EXCEPTION_ILLEGAL_FUNCTION
				self->pbackend->respondException(self->pbackend,
						self->slaveId,
						function,
						MODBUS_EXCEPTION_ILLEGAL_FUNCTION) ;
				self->sendRespond = true;
				break;
			}

		//call handler
		self->receiveHandler();
	}
	else{
		// address doesn't match do not respond to broadcast
	}
}
/* Sets many bits from a table of bytes (only the bits between idx and
   idx + nb_bits are set) */
void _modbus_set_bits_from_bytes(uint8_t *dest, int16_t idx, uint16_t nb_bits,
                                const uint8_t *tab_byte){
    uint16_t i;
    int16_t shift = 0;

    for (i = idx; i < idx + nb_bits; i++) {
        dest[i] = tab_byte[(i - idx) / 8] & (1 << shift) ? 1 : 0;
        /* gcc doesn't like: shift = (++shift) % 8; */
        shift++;
        shift %= 8;
    }
}

