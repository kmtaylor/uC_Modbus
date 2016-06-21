/* Copyright (C) 2016 Kim Taylor
 *
 * This module holds non-portable code for the Modbus library.
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

static uint8_t _modbus_timer_finished = 0;

CY_ISR(modbus_timeout) {
    _modbus_timer_finished = 1;
}

void _modbus_timer_stop_and_reset(void) {
    MODBUS_TIMER_Stop();
    /* Reset must be configured for pulse operation */
    MODBUS_TIMER_RESET_Write(1);
    _modbus_timer_finished = 0;
}

void _modbus_timer_start(void) {
    MODBUS_TIMER_Enable();
}

void _modbus_timer_init(void) {
    MODBUS_TIMER_Init();
    MODBUS_TIMEOUT_StartEx(modbus_timeout);
}
