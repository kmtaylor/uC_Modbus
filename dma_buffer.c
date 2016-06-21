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

#ifdef DEBUG_DMA
#include <stdio.h>
#include <stdint.h>
#else
#include <project.h>
#include "stdint.h"
#endif

#include "dma_buffer.h"

void buf_in(struct buf_s *buffer, uint8_t val) {
    buffer->count[buffer->base]++;
    buffer->_data[buffer->base][buffer->index[buffer->base]++] = val;
}

uint8_t buf_out(struct buf_s *buffer) {
    buffer->count[buffer->base]--;
    return buffer->_data[buffer->base][buffer->index[buffer->base]++];
}

void buf_clear(struct buf_s *buffer, uint8_t base) {
    buffer->count[base] = 0;
    buffer->index[base] = 0;
}

/* Must be called with interrupts disabled.
 * Switches the buffer that is to be written by buf_in
 * Returns the address of a buffer that may be transferred via DMA */
uint8_t *buf_switch(struct buf_s *buffer, uint8_t *size) {
    uint8_t old_base = buffer->base;

    if (size) *size = buffer->index[old_base];
    buf_clear(buffer, old_base);
    buffer->base = !buffer->base;

    return buffer->_data[old_base];
}

/* A race condition exists here. The DMA controller will start writing into the
 * returned buffer, while an interrupt might start reading from it. We rely on
 * the fact that the DMA controller will be faster than the interrupt reads.
 * Ensure that the DMA controller has been fully set up before enabling
 * interrupts.
 * The PSoC API seems to ignore this as USBUART_ReadOutEP returns immediately */
uint8_t *buf_get(struct buf_s *buffer) {
    if (!buffer->count[0]) return buffer->_data[0];
    if (!buffer->count[1]) return buffer->_data[1];
    return NULL;
}

static uint8_t which_buf(struct buf_s *buffer, uint8_t *dma_buf) {
    return dma_buf == buffer->_data[1] ? 1 : 0;
}

void buf_update(struct buf_s *buffer, uint8_t *dma_buf,
		uint8_t *start, uint8_t size) {
    uint8_t base = which_buf(buffer, dma_buf);
    buffer->index[base] = start - buffer->_data[base];
    buffer->count[base] = size;
}

uint8_t buf_empty(struct buf_s *buffer, uint8_t *dma_buf) {
    uint8_t base = which_buf(buffer, dma_buf);
    return !buffer->count[base];
}

uint8_t buf_full(struct buf_s *buffer) {
    return buffer->index[buffer->base] == USB_BUFFER_SIZE;
}

uint8_t buf_empty_switch(struct buf_s *buffer) {
    /* First try and switch buffers */
    if (!buffer->count[buffer->base])
	buf_switch(buffer, NULL);

    return !buffer->count[buffer->base];
}
