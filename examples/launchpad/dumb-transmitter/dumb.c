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
 *         Periodic broadcast example
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "dev/cc2500.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* time between transmissions */
#define TRANSMISSION_INTERVAL     (CLOCK_SECOND)

/* Channels; any number from 129..65535 (0..128 are reserved) */
#define UNICAST_CONTROL_CH        4011
#define BROADCAST_CONTROL_CH      4012

/* send broadcast (1) or unicast (0) */
#define SEND_BC                   0

/* some defines for the packets to send... */
#define MSGMIN  10
#define MSGMAX  30
/*---------------------------------------------------------------------------*/
static struct unicast_conn uc;
static struct broadcast_conn bc;

static uint8_t msglen = MSGMIN;
static const uint8_t msg[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
     14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
     31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
     48, 49, 50, 51, 52, 53, 54, 55};
/*---------------------------------------------------------------------------*/
PROCESS(hello_process, "Dumb txer process");
AUTOSTART_PROCESSES(&hello_process);
/*---------------------------------------------------------------------------*/
/* the callback structs - first *receive*-callback, then *sent*-callback for
    these connections */
#if SEND_BC
static void
bcr(struct broadcast_conn *c, const rimeaddr_t *f)
{
  PRINTF("BBBC received!\n");
}
/*---------------------------------------------------------------------------*/
static void
bcs(struct broadcast_conn *c, int status, int num_tx)
{
  PRINTF("BBBC sent!\n");
}
/*---------------------------------------------------------------------------*/
static struct broadcast_callbacks bccb = {bcr, bcs};
#else /* SEND_BC */
/*---------------------------------------------------------------------------*/
static void
ucr(struct unicast_conn *c, const rimeaddr_t *f)
{
  PRINTF("UC received!\n");
}
/*---------------------------------------------------------------------------*/
static void
ucs(struct unicast_conn *c, int status, int num_tx)
{
  PRINTF("UC sent!\n");
}
/*---------------------------------------------------------------------------*/
static struct unicast_callbacks uccb = {ucr, ucs};
#endif /* SEND_BC */
/*---------------------------------------------------------------------------*/
/* This process holds a broadcast connection and transmits a periodic message */
static struct etimer transmission_et;
PROCESS_THREAD(hello_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER(unicast_close(&uc););
  PROCESS_BEGIN();

  /* open a connection on a specified channel, much like ports in TCP/UDP */
  #if SEND_BC
  broadcast_open(&bc, BROADCAST_CONTROL_CH, &bccb);
  #else   /* SEND_BC */
  unicast_open(&uc, UNICAST_CONTROL_CH, &uccb);
  #endif  /* SEND_BC */

  if(rimeaddr_node_addr.u8[0] == 1) {
    while(1) {
      rimeaddr_t dest;
      dest.u8[0] = 2;
      dest.u8[1] = 0;
      packetbuf_copyfrom(msg, msglen);
      #if SEND_BC
      broadcast_send(&bc);
      #else   /* SEND_BC */
      unicast_send(&uc, &dest);
      #endif  /* SEND_BC */
      etimer_set(&transmission_et, TRANSMISSION_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&transmission_et));
    }
  } else {
    PROCESS_WAIT_EVENT_UNTIL(0);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
