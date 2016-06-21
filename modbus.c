/*
 * Copyright 2016 Kim Taylor
 * Based on libmodbus by Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <project.h>

#include "stdint.h"
#include "modbus-local.h"
#include "modbus.h"

static uint8_t modbus_msg[MODBUS_MAX_PACKET_LENGTH];
#if MODBUS_USE_FUNCTION_POINTERS
static modbus_write_t	    _modbus_write;
static modbus_read_ready_t  _modbus_read_ready;
static modbus_read_t	    _modbus_read;
static modbus_process_t	    _modbus_process;
static modbus_forward_t	    _modbus_forward;
#else
#define _modbus_write	    MODBUS_WRITE_FUNC
#define _modbus_read_ready  MODBUS_READ_READY_FUNC
#define _modbus_read	    MODBUS_READ_FUNC
#define _modbus_process	    MODBUS_PROCESS_FUNC
#define _modbus_forward	    MODBUS_FORWARD_FUNC
int8_t _modbus_process(uint8_t function, uint8_t *msg);
#endif

#include "modbus-psoc.c"

static void (*_tx_led_enable)(uint8_t val) = NULL;
static void (*_rx_led_enable)(uint8_t val) = NULL;
static uint8_t _tx_led_enable_val;
static uint8_t _rx_led_enable_val;
static uint8_t _slave_addr;

#define CRC16_POLY 0xA001
#define CRC16_INIT 0xFFFF
#define CRC_0(crc) (crc & 0xff)
#define CRC_1(crc) (crc >> 8)

#define MODBUS_SLAVE_OFFSET     0
#define MODBUS_FUNCTION_OFFSET  1

static uint16_t crc16_update(uint16_t crc, uint8_t _data) {
    uint8_t i;

    crc ^= _data;

    for (i = 0; i < 8; i++) {
	if (crc & 1) crc = (crc >> 1) ^ CRC16_POLY;
	else crc >>= 1;
    }

    return crc;
}

static uint16_t crc16_bytes(uint8_t *_data, uint8_t bytes) {
    uint16_t crc = CRC16_INIT;
    while (bytes--) {
	crc = crc16_update(crc, *(_data++));
    }
    return crc;
}

#define HEADER_FUNCTION_LENGTH	2
#define CRC_LENGTH		2
/* 3 steps are used to parse the query */
typedef enum {
    _STEP_FUNCTION,
    _STEP_META,
    _STEP_DATA,
} _step_t;

typedef enum {
    MSG_INDICATION,
    MSG_CONFIRMATION,
} msg_type_t;

static uint8_t compute_meta_length_after_function(
                uint8_t function, msg_type_t msg_type) {
    uint8_t length;

    if (msg_type == MSG_INDICATION) {
        if (function <= _FC_WRITE_SINGLE_REGISTER) {
            length = 4;
        } else if (function == _FC_WRITE_MULTIPLE_COILS ||
                   function == _FC_WRITE_MULTIPLE_REGISTERS) {
            length = 5;
        } else if (function == _FC_MASK_WRITE_REGISTER) {
            length = 6;
        } else if (function == _FC_WRITE_AND_READ_REGISTERS) {
            length = 9;
        } else {
            /* _FC_READ_EXCEPTION_STATUS, _FC_REPORT_SLAVE_ID */
            length = 0;
        }
#if MODBUS_PROCESS_CONFIRMATION
    } else {
        /* MSG_CONFIRMATION */
        switch (function) {
        case _FC_WRITE_SINGLE_COIL:
        case _FC_WRITE_SINGLE_REGISTER:
        case _FC_WRITE_MULTIPLE_COILS:
        case _FC_WRITE_MULTIPLE_REGISTERS:
            length = 4;
            break;
        case _FC_MASK_WRITE_REGISTER:
            length = 6;
            break;
        default:
            length = 1;
        }
#endif
    }

    return length;
}

static uint8_t compute_data_length_after_meta(
                uint8_t *modbus_msg, msg_type_t msg_type) {
    uint8_t function = modbus_msg[1];
    uint8_t length;

    if (msg_type == MSG_INDICATION) {
        switch (function) {
        case _FC_WRITE_MULTIPLE_COILS:
        case _FC_WRITE_MULTIPLE_REGISTERS:
            length = modbus_msg[6];
            break;
        case _FC_WRITE_AND_READ_REGISTERS:
            length = modbus_msg[10];
            break;
        default:
            length = 0;
        }
#if MODBUS_PROCESS_CONFIRMATION
    } else {
        /* MSG_CONFIRMATION */
        if (function <= _FC_READ_INPUT_REGISTERS ||
            function == _FC_REPORT_SLAVE_ID ||
            function == _FC_WRITE_AND_READ_REGISTERS) {
            length = modbus_msg[2];
        } else {
            length = 0;
        }
#endif
    }

    length += CRC_LENGTH;

    return length;
}

static void modbus_reply(int8_t bytes) { 
    uint8_t function;
    uint16_t crc;

    if (_tx_led_enable) _tx_led_enable(_tx_led_enable_val);
    function = modbus_msg[MODBUS_FUNCTION_OFFSET];

    bytes -= HEADER_FUNCTION_LENGTH;
    if ((bytes = _modbus_process(function,
			    modbus_msg + HEADER_FUNCTION_LENGTH)) < 0) {
	function |= MODBUS_EXCEPTION;
	modbus_msg[HEADER_FUNCTION_LENGTH] = MODBUS_EXCEPTION_ILLEGAL_FUNCTION;
	bytes = 1;
    }
    bytes += HEADER_FUNCTION_LENGTH;

    modbus_msg[MODBUS_FUNCTION_OFFSET] = function;
    crc = crc16_bytes(modbus_msg, bytes);
    modbus_msg[bytes++] = CRC_0(crc);
    modbus_msg[bytes++] = CRC_1(crc);
    _modbus_write(modbus_msg, bytes);
    if (_tx_led_enable) _tx_led_enable(!_tx_led_enable_val);
}

uint8_t modbus_poll(void) {
    static uint8_t length_to_read = HEADER_FUNCTION_LENGTH;
    static uint8_t msg_length = 0;
    static _step_t step = _STEP_FUNCTION;
    static msg_type_t msg_type = MSG_INDICATION;
    uint8_t retval = 0;
    uint16_t crc;
    uint8_t slave;

    if (msg_length && _modbus_timer_finished) goto reset;
    
    if (!_modbus_read_ready()) return 0;
    
    modbus_msg[msg_length++] = _modbus_read();
    length_to_read--;

    if (msg_length == 1) {
	_modbus_timer_start();
	if (_rx_led_enable) _rx_led_enable(_rx_led_enable_val);
    }

    if (!length_to_read) {
        switch (step) {
            case _STEP_FUNCTION:
                /* Function code position */
                length_to_read = compute_meta_length_after_function(
				modbus_msg[1], msg_type);
                if (length_to_read != 0) {
                    step = _STEP_META;
                    break; 
                } /* else switches straight to the next step */
            case _STEP_META:
                length_to_read = compute_data_length_after_meta(modbus_msg,
				msg_type);
                if ((msg_length + length_to_read) > MODBUS_MAX_PACKET_LENGTH)
		    goto reset;
                step = _STEP_DATA;
                break; 
            default:
                break;
        }       
    }

    if (!length_to_read) {
        /* All data collected by here */
	crc = crc16_bytes(modbus_msg, msg_length);
	if (!crc) retval = msg_length - CRC_LENGTH;
	else goto reset;

	/* If the slave address does not match this device, expect and ignore a
	 * confirmation message from another device on the network. */
	if (msg_type == MSG_CONFIRMATION) goto reset;

	slave = modbus_msg[MODBUS_SLAVE_OFFSET];
	if (slave != _slave_addr) {
#if MODBUS_FORWARD_PACKETS
	    /* Forward this packet on to the next interface */
	    _modbus_forward(modbus_msg, msg_length);
	    goto reset;
#endif
	    /* Leave the timer running so that a reset will occur if no
	     * device ever responds */
	    msg_type = MSG_CONFIRMATION;
	    goto reset_with_timeout;
	}

	modbus_reply(retval);

reset:
	_modbus_timer_stop_and_reset();
	msg_type = MSG_INDICATION;
reset_with_timeout:
	msg_length = 0;
	length_to_read = HEADER_FUNCTION_LENGTH;
	step = _STEP_FUNCTION;
	if (_rx_led_enable) _rx_led_enable(!_rx_led_enable_val);
    }   
    
    return retval;
}

#if MODBUS_USE_FUNCTION_POINTERS
void modbus_init(
	    uint8_t		slave_addr,
            modbus_write_t      modbus_write,
            modbus_read_ready_t modbus_read_ready,
            modbus_read_t       modbus_read,
            modbus_process_t    modbus_process) {
    _modbus_write	= modbus_write;
    _modbus_read_ready	= modbus_read_ready;
    _modbus_read	= modbus_read;
    _modbus_process	= modbus_process;
#else
void modbus_init(uint8_t slave_addr) {
#endif
    _slave_addr = slave_addr;
    _modbus_timer_init();
}

void modbus_tx_led(void (*led_enable)(uint8_t val), uint8_t val) {
    _tx_led_enable = led_enable;
    _tx_led_enable_val = val;
}

void modbus_rx_led(void (*led_enable)(uint8_t val), uint8_t val) {
    _rx_led_enable = led_enable;
    _rx_led_enable_val = val;
}

/* Helper functions */
int8_t modbus_slave_id_response(uint8_t *msg) {
    msg[0] = sizeof(MODBUS_SLAVE_STRING) + 2;
    msg[1] = _slave_addr;
    msg[2] = 0xff; /* Run indicator status */
    memcpy(&msg[3], MODBUS_SLAVE_STRING, sizeof(MODBUS_SLAVE_STRING));
    return sizeof(MODBUS_SLAVE_STRING) + 3;
}

/* Iterators */
uint16_t mb_address;
uint8_t	mb_nr_regs;
uint8_t mb_resp_bytes;
static uint8_t _mb_data_offset;

void mb_data_init(uint8_t *msg, uint8_t fn) {
    mb_address = mb_buf_to_val(&msg[MODBUS_MSG_ADDR_OFFSET]);
    mb_nr_regs = mb_buf_to_val(&msg[MODBUS_MSG_NR_REGS_OFFSET]);
    switch (fn) {
	case _FC_READ_HOLDING_REGISTERS:
	    _mb_data_offset = 1;
	    mb_resp_bytes = 1 + mb_nr_regs * 2;
	    msg[MODBUS_MSG_LEN_OFFSET] = mb_resp_bytes - 1;
	    return;
	case _FC_WRITE_SINGLE_REGISTER:
	    _mb_data_offset = 2;
	    mb_nr_regs = 1;
	    return;
	case _FC_WRITE_MULTIPLE_REGISTERS:
	    _mb_data_offset = 5;
	    return;
    }
}

void mb_data_resp(uint8_t *msg, uint16_t val) {
    _mb_data_offset += mb_val_to_buf(&msg[_mb_data_offset], val);
    mb_nr_regs--;
    mb_address++;
}

uint16_t mb_data_next(uint8_t *msg) {
    uint16_t retval = mb_buf_to_val(&msg[_mb_data_offset]);
    mb_nr_regs--;
    mb_address++;
    _mb_data_offset += 2;
    return retval;
}
