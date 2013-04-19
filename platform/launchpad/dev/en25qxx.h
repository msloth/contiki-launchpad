/*
 * Copyright (c) 2012
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
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/**
 * \file
 *    en25qxx.c
 * \author
 *    Marcus Lunden <marcus.lunden@gmail.com>
 * \desc
 *    driver for the EN25Q{64, 128 etc} serial flash
 *    This driver is intended just for basic flash read/write/format functional-
 *    ity, all the rest (any file system etc) happens above this driver.
 *    
 *    
 *    
 *    
 *    
 */

#ifndef __EN25QXX_H__
#define __EN25QXX_H__

#include <stdio.h>
#include <stdint.h>
#include "contiki.h"

/*--------------------------------------------------------------------------*/
/* init the serial flash */
int         en25qxx_init(void);

/* write a single byte at an address */
void        en25qxx_write_byte(uint32_t dest, uint8_t wd);

/* write a series of bytes to flash */
uint32_t    en25qxx_write_burst(uint32_t dest, uint8_t *srcbuf, uint32_t len);

/* read a single byte from an address */
uint8_t     en25qxx_read_byte(uint32_t from);

/* read a series of bytes from flash */
uint32_t    en25qxx_read_burst(uint32_t from, uint8_t *destbuf, uint32_t len);

/* seek to address */
uint8_t     en25qxx_seek(uint32_t from);

/* read from address prev seek'ed to */
uint8_t     en25qxx_read(void);
/*--------------------------------------------------------------------------*/
/* format a page of the entire flash */
int en25qxx_format_page(int pno);

/* format the entire flash */
int en25qxx_format_all(void);
/*--------------------------------------------------------------------------*/

#endif /* __EN25QXX_H__ */

