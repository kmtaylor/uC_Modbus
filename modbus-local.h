/* Copyright (C) 2016 Kim Taylor
 *
 * Customise this module for your Modbus implementation
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

#define MODBUS_SLAVE_STRING "USB-Modbus Bridge"

#define MODBUS_MAX_PACKET_LENGTH	64

#define MODBUS_PROCESS_CONFIRMATION	1
#define MODBUS_USE_FUNCTION_POINTERS	0
#define MODBUS_FORWARD_PACKETS		1

#define MODBUS_WRITE_FUNC	USBFS_PutData
#define MODBUS_READ_READY_FUNC	usb_read_ready
#define MODBUS_READ_FUNC	usb_get_byte
#define MODBUS_PROCESS_FUNC	modbus_respond
#define MODBUS_FORWARD_FUNC	RS485_PutArray

uint8_t usb_read_ready(void);
uint8_t usb_get_byte(void);

