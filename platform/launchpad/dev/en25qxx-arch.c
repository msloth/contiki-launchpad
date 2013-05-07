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
 *         EN25Qxx platform specific drivers
 * \author
 *         Marcus Linderoth, <linderoth.marcus@gmail.com>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-net.h"
#include "dev/en25qxx.h"
/*--------------------------------------------------------------------------*/
 /* routines for bit-banged SPI (in order to save the SPI USCI for ) */
 void
 en25qxx_arch_spibb_enable(void)
 {

 }
/*--------------------------------------------------------------------------*/
 void
 en25qxx_arch_spibb_disable(void)
 {

 }
/*--------------------------------------------------------------------------*/
 void
 en25qxx_arch_spibb_write_single(void)
 {

 }
/*--------------------------------------------------------------------------*/
 void
 en25qxx_arch_spibb_write_burst(void)
 {

 }
/*---------------------------------------------------------------------------*/
/* init irq and spi enable pins; spi pins are inited in spi_init() */
void
en25qxx_arch_init(void)
{
  /* init SPI CS pin; active low, output */
  EN25QXX_CSN_PORT(SEL)  &= ~EN25QXX_SPI_CSN_PIN;
  EN25QXX_CSN_PORT(SEL2) &= ~EN25QXX_SPI_CSN_PIN;
  EN25QXX_CSN_PORT(DIR)  |= EN25QXX_SPI_CSN_PIN;
  EN25QXX_CSN_PORT(OUT)  |= EN25QXX_SPI_CSN_PIN;

  /* init spi */
  spi_init();
  EN25QXX_SPI_DISABLE();
}
/*--------------------------------------------------------------------------*/

