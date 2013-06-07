/*
 * Copyright (c) 2013, Marcus Linderoth, http://forfunandprof.it
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _SIMPLE_SERIAL_H_
#define _SIMPLE_SERIAL_H_

#include <stdint.h>
/*---------------------------------------------------------------------------*/
/*
 * This is Simple-serial, a simplified serial output module for use when
 * printf is too much for any reason. Used eg in Launchpad when including
 * radio as the netstack and packetbuf needs a lot of RAM, hence stack
 * overflow when trying to printf in callbacks. Simple-serial is output only.
 *
 * Simple-serial does not interfere with regular printf output.
 */
/*---------------------------------------------------------------------------*/
/* Init the UART */
void simple_serial_init(void);

/* Print a string. You can use \n and \t as usual */
void simple_serial_print_str(char *s);

/* print a string, and an uint16_t. If %u is in the string, that's where the int will go */
void simple_serial_print_strint(char *s, uint16_t i);

/* print an uint8_t in hex */
void simple_serial_print_hex_u8(uint8_t hu);
/* print an uint16_t in hex */
void simple_serial_print_hex_u16(uint16_t hu);
/* print an uint32_t in hex */
void simple_serial_print_hex_u32(uint32_t hu);

/* print an uint8_t */
void simple_serial_print_u8(uint8_t u);
/* print an uint16_t */
void simple_serial_print_u16(uint16_t u);
/* print an uint32_t */
void simple_serial_print_u32(uint32_t u);

/* print a raw byte stream - allows for binary output, not forced to ASCII */
void simple_serial_print_raw(uint8_t *u, uint8_t len);
/*---------------------------------------------------------------------------*/
#endif  /* _SIMPLE_SERIAL_H_ */
