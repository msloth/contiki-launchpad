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
 *         CC2500 platform specific drivers
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include "contiki.h"
#include "contiki-net.h"
#include "dev/spi.h"
#include "dev/cc2500.h"
#include "isr_compat.h"

/*
 * This file handles two things: the ISR for the radio and initiating the radio
 * on mcu (SPI, pins, irq). The cc2500.c handles initiating the actual radio
 * with the proper settings.
 */
/*--------------------------------------------------------------------------*/
/* The interrupt service routine for when the radio signals received a packet */
ISR(PORT2, cc2500_port2_interrupt)
{
  /* check for a valid packet, and if there is one, we wake up the mcu */
  if(P2IFG & CC2500_GDO_PIN) {
    P2IFG &= ~CC2500_GDO_PIN;
    if(cc2500_interrupt()) {
      LPM4_EXIT;
    }
  }
}
/*--------------------------------------------------------------------------*/
/* init irq and spi enable pins; spi pins are inited in spi_init() */
void
cc2500_arch_init(void)
{
  /* init SPI CS pin; active low, output */
  CC2500_CSN_PORT(SEL) &= ~CC2500_SPI_CSN_PIN;
  CC2500_CSN_PORT(SEL2) &= ~CC2500_SPI_CSN_PIN;
  CC2500_CSN_PORT(DIR) |= CC2500_SPI_CSN_PIN;
  CC2500_CSN_PORT(OUT) |= CC2500_SPI_CSN_PIN;

  /* init spi */
  spi_init();
  CC2500_SPI_DISABLE();

  /* init GDO interrupt pin */
  CC2500_GDO_PORT(SEL) &= ~CC2500_GDO_PIN;
  CC2500_GDO_PORT(SEL2) &= ~CC2500_GDO_PIN;
  CC2500_GDO_PORT(DIR) &= ~CC2500_GDO_PIN;
  CC2500_GDO_PORT(IE) |= CC2500_GDO_PIN;
  CC2500_GDO_PORT(IES) |= CC2500_GDO_PIN;   /* 1 = high->low transition (EOP) */
}
/*--------------------------------------------------------------------------*/

