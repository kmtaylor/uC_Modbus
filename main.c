/* Copyright (C) 2016 Kim Taylor
 *
 * Example application acts as a bridge between USB and modbus. Tested using
 * Cypress CY8C3245-PVI150
 *
 * hbc_mac is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * hbc_mac is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with hbc_mac.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <project.h>

#include "stdint.h"
#include "modbus-local.h"
#include "modbus.h"
#include "dma_buffer.h"

#define MODBUS_SLAVE_ADDR 20

static struct buf_s wr_buf;
static volatile struct buf_s rd_buf;

static void usb_poll(void) {
    if (USBFS_GetConfiguration()) {
	if (!usb_up) USBFS_CDC_Init();
	usb_up = 1;
    } else {
	usb_up = 0;
    }
}

uint8_t usb_get_byte(void) {
    return buf_out(&wr_buf);
}

uint8_t usb_read_ready(void) {
    return !buf_empty_switch(&wr_buf);
}

static uint16_t test_var;

static int8_t modbus_write_regs(uint8_t *msg, uint8_t function) {
    mb_data_init(msg, function);

    if (mb_nr_regs > MAX_NR_REGS(_FC_WRITE_MULTIPLE_REGISTERS)) return -1;

    while (mb_nr_regs) {
	switch (mb_address) {
	    case 0:
		test_var = mb_data_next(msg);
		break;

	    default:
		return -1;
	}
    }

    return MODBUS_WRITE_RESP_SIZE;
}

static int8_t modbus_read_regs(uint8_t *msg) {
    mb_data_init(msg, _FC_READ_HOLDING_REGISTERS);

    if (mb_nr_regs > MAX_NR_REGS(_FC_READ_HOLDING_REGISTERS)) return -1;

    while (mb_nr_regs) {
	switch (mb_address) {
	    case MB_VERSION:
		if (mb_nr_regs < MB_VERSION_REGS) return -1;
		mb_data_resp(msg, GIT_REVISION >> 16);
		mb_data_resp(msg, GIT_REVISION);
		break;

	    case 0:
		mb_data_resp(msg, test_var);
		break;

	    default:
		return -1;
	}
    }

    return mb_resp_bytes;
}

int8_t modbus_respond(uint8_t function, uint8_t *msg) {
    switch (function) {
        case _FC_WRITE_SINGLE_REGISTER:
        case _FC_WRITE_MULTIPLE_REGISTERS:
            return modbus_write_regs(msg, function);
        case _FC_READ_HOLDING_REGISTERS:
            return modbus_read_regs(msg);
        case _FC_REPORT_SLAVE_ID:
            return modbus_slave_id_response(msg);
    }
    return -1;
}

CY_ISR(rs485_rx_isr) {
    while (RS485_GetRxBufferSize()) {
	if (buf_full(&rd_buf)) break;
	buf_in(&rd_buf, RS485_ReadRxData());
    }
}

static void usb_to_modbus(void) {
    uint8_t bytes;
    uint8_t *dma_buf;

    if (usb_up && (bytes = USBFS_GetCount())) {
	if ((dma_buf = buf_get(&wr_buf))) {
	    USBFS_GetData(dma_buf, bytes);
	    buf_update(&wr_buf, dma_buf, dma_buf, bytes);
	    /* Packets not destined for this slave are automatically 
	     * forwarded */
	}
    }
}

static void rs485_to_usb(void) {
    static uint8_t bytes = 0;
    static uint8_t *dma_buf;

    if (!bytes) {
	RS485_RX_ISR_Disable();

	if (buf_full(&rd_buf)) RS485_RX_ISR_SetPending();
	dma_buf = buf_switch(&rd_buf, &bytes);

	RS485_RX_ISR_Enable();
    }

    if (bytes) {
	if (!USBFS_CDCIsReady()) return;
	USBFS_PutData(dma_buf, bytes);
	bytes = 0;
    }
}

int main() {
    USBFS_Start(1, USBFS_DWR_VDDD_OPERATION);
    CyGlobalIntEnable;

    RS485_Start();
    RS485_RX_ISR_StartEx(rs485_rx_isr);
    BUTTON_ISR_StartEx(button_isr);

    modbus_init(MODBUS_SLAVE_ADDR);
    modbus_tx_led(STATUS_LED_0_Write, MODBUS_ACTIVE_HIGH);
    modbus_rx_led(STATUS_LED_1_Write, MODBUS_ACTIVE_HIGH);

    while (1) {

	/* Store USB data in buffer for Modbus */
	usb_to_modbus();

	/* Forward RS485 bus data to Modbus master */
	rs485_to_usb();

	modbus_poll();
	usb_poll();
    }
}
