/*
 * Copyright (c) 2006, Swedish Institute of Computer Science
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "sys/autostart.h"
#include "dev/button.h"
#include "dev/adc.h"

#include "netstack.h"

#include "node-id.h"
#include "lib/random.h"


#if DCOSYNCH_CONF_ENABLED
#undef DCOSYNCH_CONF_ENABLED
#define DCOSYNCH_CONF_ENABLED 0
#endif
#define WITH_UIP 0

/*--------------------------------------------------------------------------*/
#if SET_NODE_ID
static void
set_rime_addr(void)
{
  rimeaddr_t addr;
  int i;

  memset(&addr, 0, sizeof(rimeaddr_t));
  if(node_id == 0) {
    for(i = 0; i < sizeof(rimeaddr_t); ++i) {
      addr.u8[i] = ds2411_id[7 - i];
    }
  } else {
    addr.u8[0] = node_id & 0xff;
    addr.u8[1] = node_id >> 8;
  }
  rimeaddr_set_node_addr(&addr);
  printf("Rime started with address ");
  for(i = 0; i < sizeof(addr.u8) - 1; i++) {
    printf("%d.", addr.u8[i]);
  }
  printf("%d\n", addr.u8[i]);
}
#endif

/*---------------------------------------------------------------------------*/
/*  P1SEL &=~ (LEDS_RED | LEDS_GREEN);*/
/*  P1DIR |= (LEDS_RED | LEDS_GREEN);*/
/*  P1OUT |= (LEDS_RED);*/
/*  P1OUT |= (LEDS_GREEN);*/
/*  while (1) { }*/
/*#define LON(x)   (P1OUT |= (x))*/
/*#define LOFF(x)   (P1OUT &=~ (x))*/

int
main(int argc, char **argv)
{
  msp430_cpu_init();
  leds_init();
  leds_on(LEDS_ALL);
  clock_init();


  #if USE_SERIAL
  /* 
   * The Launchpad is limited to 9600 by the msp430-usb bridge; higher speeds can
   * be used with a separate uartserial->usb cable connected to the rxtx pins on
   * the header, but this is kept at a maximum simplicity now, hence 9600.
   */
  uart1_init(BAUD2UBR(9600));
  #endif

  rtimer_init();
  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();

  button_init();
  adc_init();

  #if USE_SERIAL
  uart1_set_input(serial_line_input_byte);
  serial_line_init();
  #endif


/*  #if USE_RIME */
  #if 1
  cc2500_init();
  cc2500_set_channel(0);
/*  cc2500_set_channel(RF_CHANNEL);*/

/*  set_rime_addr();*/
/*  if(node_id > 0) {*/
/*    printf("Node id is set to %u.\n", node_id);*/
/*  } else {*/
/*    printf("Node id is not set.\n");*/
/*  }*/

  netstack_init();
      /*  NETSTACK_RADIO.init();*/
      /*  NETSTACK_RDC.init();*/
      /*  NETSTACK_MAC.init();*/
      /*  NETSTACK_NETWORK.init();*/
  #endif

  printf(CONTIKI_VERSION_STRING " started. ");
  watchdog_start();
  autostart_start(autostart_processes);

  /* The Contiki main loop, greatly simplified and shortened compared with eg
   * msp430f1611 due to severe memory constraints (mainly RAM) */
  
  
  /* ugly temporary fix until I find what is setting the P1.6 (green LED) to sth else */
  #define L_RED    (1)
  #define L_GREEN  (1<<6)
  P1SEL &= ~(L_RED | L_GREEN);
  P1SEL2 &= ~(L_RED | L_GREEN);
  P1DIR |= (L_RED | L_GREEN);
  P1OUT &= ~(L_RED | L_GREEN);

  leds_off(LEDS_ALL);
  while(1) {
    int r;

    do {
      watchdog_periodic();
      r = process_run();
    } while(r > 0);

    #if USE_SERIAL
      if(process_nevents() == 0 && !uart1_active()) {
    #else
      if(process_nevents() == 0) {
    #endif    
      LPM3;   /* LPM4? No, due to SMCLK driving clock (?) */
    }
  }
  return 0;
}
/*---------------------------------------------------------------------------*/

