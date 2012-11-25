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
 *         Launchpad main file
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "contiki.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/uart0.h"
#include "dev/watchdog.h"
#include "sys/autostart.h"
#include "lib/random.h"
#include "net/rime.h"
#include "netstack.h"
#include "dev/button.h"
#include "dev/adc.h"
#include "dev/pwm.h"

/*---------------------------------------------------------------------------*/
/*  P1SEL &=~ (LEDS_RED | LEDS_GREEN);*/
/*  P1DIR |= (LEDS_RED | LEDS_GREEN);*/
/*  P1OUT |= (LEDS_RED);*/
/*  P1OUT |= (LEDS_GREEN);*/
/*  while (1) { }*/
/*#define LON(x)   (P1OUT |= (x))*/
/*#define LOFF(x)   (P1OUT &=~ (x))*/

void
main(void)
{
  msp430_cpu_init();
  leds_init();
  leds_on(LEDS_ALL);
  clock_init();

  #if USE_SERIAL
  /* 
   * The Launchpad is limited to 9600 by the msp430--usb bridge; higher speeds can
   * be used with a separate uartserial->usb cable connected to the rxtx pins on
   * the header, but this Contiki port is kept at a maximum simplicity now, hence
   * 9600. If you do get one such cable, you can increase this to eg 115200.
   */
  uart0_init(BAUD2UBR(9600));
  #else
  /*
   * any printf's makes compiler complain of unresolved references to putchar;
   * this solves that. Must come before first printf.
   */
  #define printf(...)
  #endif  /* USE_SERIAL */

  rtimer_init();
  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();

  button_init();
  adc_init();

  #if _MCU_ == 2553
  #if HAS_EXT_OSC
  /* pwm only available on 2553 with external crystal as it has two hw timers */
  pwm_init();
  #endif    /* HAS_EXT_OSC */
  #endif    /* _MCU_ == 2553 */

  #if USE_SERIAL
  uart0_set_input(serial_line_input_byte);
  serial_line_init();
  #endif  /* USE_SERIAL */


  #if USE_RADIO
  {
    rimeaddr_t addr;
    uint8_t i;
    /* Check that Magic number and node id first byte are correct */
    if (NODEID_INFOMEM_LOCATION[0] != 0xBE || NODEID_INFOMEM_LOCATION[1] != 0xEF) {
      /* error, just set to fail-address */
      addr.u8[0] = 0xde;    // 222
      addr.u8[1] = 0xad;    // 173
    } else {
      addr.u8[0] = NODEID_INFOMEM_LOCATION[2];
      addr.u8[1] = NODEID_INFOMEM_LOCATION[3];
    }
    rimeaddr_set_node_addr(&addr);
    printf("Rime started with address ");
    for(i = 0; i < sizeof(addr.u8) - 1; i++) {
      printf("%d.", addr.u8[i]);
    }
    printf("%d\n", addr.u8[i]);
  }

  netstack_init();

  //XXX sth in this clause (radio driver?) messes with LEDs; find and fix
  leds_init();  // XXX remove when fixed.
  #endif  /* USE_RADIO */

  watchdog_start();
  autostart_start(autostart_processes);
  leds_off(LEDS_ALL);
  printf(CONTIKI_VERSION_STRING " started. ");

  while(1) {
    /*
     * The Contiki main loop, greatly simplified and shortened compared with eg
     * msp430f1611 due to severe space constraints (mainly RAM).
     * As soon as we are not doing anything, we spend the time in LPM3.
     */
    int r;
    do {
      /* handle all events */
      watchdog_periodic();
      r = process_run();
    } while(r > 0);

    /* if not printing or pending events, sleep. */
    #if USE_SERIAL
      if(process_nevents() == 0 && !uart0_active()) {
    #else
      if(process_nevents() == 0) {
    #endif  /* USE_SERIAL */
      /* we are ready to go to sleep, LPM3 */
      LPM3;   /* No LPM4 due to SMCLK driving clock (?) */  // XXX check this LPM3/4
    }
  }
  return;
}
/*---------------------------------------------------------------------------*/

