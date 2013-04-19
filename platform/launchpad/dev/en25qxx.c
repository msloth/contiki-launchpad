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
 *    driver for the EN25Q{64, 128 etc} serial flash.
 */

#include <stdio.h>
#include <stdint.h>
#include "contiki.h"

#include "en25qxx.h"

/*---------------------------------------------------------------------------*/
/**
 * \brief      init flash and SPI for the flash
 * \return     execution result
 * \retval 0   successful
 * \retval -1  not successful
 *
 */

int
en25qxx_init(void)
{
  
}
/*---------------------------------------------------------------------------*/
/**
 * \brief           write a single byte
 * \param dest      what address to write to
 * \param wd        data byte to write
 *
 */

void
en25qxx_write_byte(uint32_t dest, uint8_t wd)
{
  
}
/*---------------------------------------------------------------------------*/
/**
 * \brief           write a stream of bytes
 * \param dest      what address to write to
 * \param srcbuf    pointer to start of buffer containing the data to write
 * \param len       number of bytes to write
 * \return          number of bytes written
 *
 */

uint32_t
en25qxx_write_burst(uint32_t dest, uint8_t *srcbuf, uint32_t len)
{
  
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief           read a single byte
 * \param from      what address to read from
 * \return          the read byte
 *
 */

uint8_t
en25qxx_read_byte(uint32_t from)
{
  
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief           read a stream of bytes
 * \param from      what address to start reading from
 * \param destbuf   pointer to start of buffer to write read data to
 * \param len       number of bytes to read
 * \return          number of bytes read
 *
 */
uint32_t
en25qxx_read_burst(uint32_t from, uint8_t *destbuf, uint32_t len)
{
  
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      seek to address
 * \param from address to seek to
 * \return     sdf
 * \retval 0   Functions that return a few specified values
 * \retval 1   can use the \retval keyword instead of \return.
 *
 */
uint8_t
en25qxx_seek(uint32_t from)
{
  
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      read from address prev seek'ed to
 * \return     the read byte
 *
 */
uint8_t
en25qxx_read(void)
{
  
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      format a page of the entire flash
 * \return     execution result
 * \retval 0   successful
 * \retval -1  not successful
 *
 */
int
en25qxx_format_page(int pno)
{
  
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief      format the entire flash
 * \return     execution result
 * \retval 0   successful
 * \retval -1  not successful
 *
 */
int
en25qxx_format_all(void)
{
  
  return 0;
}
/*---------------------------------------------------------------------------*/
