/* Copyright (C) 2016 Kim Taylor
 *
 * Ping-Pong buffer system.
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

#define USB_BUFFER_SIZE 64

struct buf_s {
    uint8_t _data[2][USB_BUFFER_SIZE];
    uint8_t index[2];
    uint8_t count[2];
    uint8_t base;
};

extern void buf_in(struct buf_s *buffer, uint8_t val);
extern uint8_t buf_out(struct buf_s *buffer);

/* Must be called with interrupts disabled.
 * Switches the buffer that is to be written by buf_in
 * Returns the address of a buffer that may be transferred via DMA */
extern uint8_t *buf_switch(struct buf_s *buffer, uint8_t *size);
extern uint8_t *buf_get(struct buf_s *buffer);
extern void buf_update(struct buf_s *buffer, uint8_t *dma_buf,
		uint8_t *start, uint8_t size);
extern uint8_t buf_empty(struct buf_s *buffer, uint8_t *dma_buf);

extern uint8_t buf_full(struct buf_s *buffer);
extern uint8_t buf_empty_switch(struct buf_s *buffer);

extern void buf_clear(struct buf_s *buffer, uint8_t base);
