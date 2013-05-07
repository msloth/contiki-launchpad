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

/**
 * \file
 *    en25qxx.c
 * \author
 *    Marcus Linderoth <linderoth.marcus@gmail.com>
 * \desc
 *    driver for the EN25Q{64, 128 etc} serial flash
 *    This driver is intended just for basic flash read/write/format functional-
 *    ity, all the rest (any file system etc) happens above this driver.
 */

#ifndef __EN25QXX_H__
#define __EN25QXX_H__

#include <stdio.h>
#include <stdint.h>
#include "contiki.h"
/*---------------------------------------------------------------------------*/
#if DOCUMENTATION

 MEMORY ORGANIZATION
 The memory is organized as:
   * 8,388,608 bytes
   * Uniform Sector Architecture
   * 128 blocks of 64-Kbyte
   * 2048 sectors of 4-Kbyte
   * 32768 pages (256 bytes each)
 Each page can be individually programmed (bits are programmed from 1 to 0).
 The device is Sector, Block or Chip Erasable but not Page Erasable.

#endif  /* if 0; commented out code */
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

/* format a 2kB sector */
int en25qxx_format_sector(int sector);

/* format the entire flash */
int en25qxx_format_all(void);
/*--------------------------------------------------------------------------*/
/*
 * commands; note that some commands are multi-byte commands and need more
 * information, this is just the first byte - refer to the datasheet.
 */
#define EN25QXX_CMD_RSTEN         (0x66)
#define EN25QXX_CMD_RST           (0x99)
#define EN25QXX_CMD_EQIO          (0x38)
#define EN25QXX_CMD_RSTEQIO       (0xff)
#define EN25QXX_CMD_WREN          (0x06)
#define EN25QXX_CMD_WRDIS         (0x05)
#define EN25QXX_CMD_READ_STATUS   (0x04)
#define EN25QXX_CMD_READ_SUSPEND  (0x09)
#define EN25QXX_CMD_WRITE_STATUS  (0x01)
#define EN25QXX_CMD_PAGE_PROGRAM  (0x02)
#define EN25QXX_CMD_WRITE_SUSPEND (0xb0)
#define EN25QXX_CMD_WRITE_RESUME  (0x30)
#define EN25QXX_CMD_SECTOR_ERASE  (0x20)
#define EN25QXX_CMD_BLOCK_ERASE   (0xd8)
#define EN25QXX_CMD_CHIP_ERASE    (0xc7)
#define EN25QXX_CMD_DEEP_SLEEP    (0xb9)
#define EN25QXX_CMD_WAKE_UP       (0xab)
#define EN25QXX_CMD_READ_MPID     (0x90)
#define EN25QXX_CMD_READ_ID       (0x9f)
#define EN25QXX_CMD_ENTER_OTP     (0x3a)
#define EN25QXX_CMD_READ_DATA        (0x03)
#define EN25QXX_CMD_FASTREAD         (0x0b)
#define EN25QXX_CMD_DUALOUT_FASTREAD (0x3b)
#define EN25QXX_CMD_DUALIO_FASTREAD  (0xbb)
#define EN25QXX_CMD_QUADOUT_FASTREAD (0xeb)
/*---------------------------------------------------------------------------*/
 #define EN25QXX_SPI_PORT(type)       P1##type
 #define EN25QXX_CSN_PORT(type)       P2##type
 #define EN25QXX_SPI_CSN_PIN          (1<<3)

 /* setting/clearing chip select help */
 #define EN25QXX_SPI_ENABLE()         (EN25QXX_CSN_PORT(OUT) &= ~EN25QXX_SPI_CSN_PIN)
 #define EN25QXX_SPI_DISABLE()        (EN25QXX_CSN_PORT(OUT) |=  EN25QXX_SPI_CSN_PIN)


/*---------------------------------------------------------------------------*/
#endif /* __EN25QXX_H__ */

