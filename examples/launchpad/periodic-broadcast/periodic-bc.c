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
 *         Periodic broadcast example
 *         For this to work you need at least two Launchpads with 2553 and external
 *         32.768 kHz oscillator and cc2500 radios. The nodes will periodically
 *         transmit a broadcast that will be received by any node that "hears" it
 *         and has opened a broadcast on that Rime channel.
 *         For addresses to work, each Launchpad must have an address burnt into
 *         it. To do so you need to connect the Launchpad, have mspdebug installed
 *         and on the path, go to contiki-launchpad/tools/launchpad/burnid and run
 *         the burnid script. Eg for node id 240.0
 *            burnid 240 0
 *         To save RAM, each address is just 2 bytes (compare with IPv6 or even
 *         IPv4 address that are much longer). This makes eg routing tables and
 *         neighbor tables smaller if you want to use/implement such.
 *         
 *         Radio communication in Rime is implemented with logical channels. The
 *         channel numbers are embedded in each frame header so that a receiver
 *         knows how to interpret the packet. Because of this, a connection must
 *         opened with a unique channel number. For each connection, you also
 *         to define callbacks, which will be called when eg a packet has been
 *         received, sent or a maximum number of re-transmissions have been
 *         exceeded. See each connection type in core/net/rime/
 *         and look at how the callback struct is defined. Eg broadcast.h:
 *             struct broadcast_callbacks {
 *               void (* recv)(struct broadcast_conn *ptr, const rimeaddr_t *sender);
 *               void (* sent)(struct broadcast_conn *ptr, int status, int num_tx);
 *             };
 *         thus you see that you need to define two callbacks, or set to NULL
 *         (see code below).
 *         
 *         When a packet is received, the contents reside in the packet buffer,
 *         packetbuf. The packetbuf_dataptr() returns a (void *) to the data.
 *         The length in bytes is given by packetbuf_datalen().
 *         Connections set 'attributes', such as time stamps, and can be read eg
 *            uint16_t hops = packetbuf_attr(PACKETBUF_ATTR_HOPS);
 *         see core/net/packetbuf.h for what attributes are possible and note
 *         that not all are set by all connections (see eg unicast.h).
 *         
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "contiki-net.h"
#include "dev/leds.h"
#include "dev/button.h"
#include "dev/cc2500.h"

/* time between transmissions */
#define TRANSMISSION_INTERVAL     (CLOCK_SECOND/4)

/* Channels; any number from 129..65535 (0..128 are reserved) */
#define BROADCAST_CH              2674
#define UNICAST_CONTROL_CH        4011
/* the connections */
static struct broadcast_conn bc;
static struct unicast_conn uc;

/* some defines for the packets to send... */
#define MSGMIN  1
#define MSGMAX  30
static const uint8_t msg[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
     14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
static uint8_t msglen = MSGMIN;
/*---------------------------------------------------------------------------*/
PROCESS(hello_process, "Hello process");
AUTOSTART_PROCESSES(&hello_process);
/*---------------------------------------------------------------------------*/
/* Broadcast receive callback; this is invoked on each broadcast received on the
Rime-channel associated to this broadcast connection, here BROADCAST_CH */

static void
bcr(struct broadcast_conn *c, const rimeaddr_t *f)
{
/*  uint8_t *buf, i, *d;*/
  /* this is unsafe as it opens up for a smash the stack attack, but that's
    quite unlikely to happen in this context. Should sanitize input and have
    an upper bound on size, now it will just print happily until first 0. */
/*  printf("[%u] Received fr %u.%u\n", clock_seconds(), f->u8[0], f->u8[1]);*/
/*  printf("[%u] Received fr %u.%u:%s\n", clock_seconds(), f->u8[0], f->u8[1], buf);*/
/*  d = packetbuf_dataptr();*/
/*  for(i = 0; i < packetbuf_datalen(); i += 1) {*/
/*    printf("%02x ", *d);*/
/*    d++;*/
/*  }*/
/*  packetbuf_copyfrom("Great! Thx!", 12);*/
/*  unicast_send(&uc, f);*/
  leds_toggle(LEDS_RED);
}
/*---------------------------------------------------------------------------*/
static void
ucr(struct unicast_conn *c, const rimeaddr_t *f)
{
  leds_toggle(LEDS_RED);
}
/*---------------------------------------------------------------------------*/
/* the callback structs; first *receive*-callback, then *sent*-callback for
    these connections */
static struct broadcast_callbacks bccb = {bcr, NULL};
static struct unicast_callbacks uccb = {ucr, NULL};
/*---------------------------------------------------------------------------*/
/* This process holds a broadcast connection and transmits a periodic message */

static struct etimer transmission_et;
PROCESS_THREAD(hello_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER(broadcast_close(&bc); unicast_close(&uc););
  PROCESS_BEGIN();

  /* open a connection on a specified channel, much like ports in TCP/UDP */
  broadcast_open(&bc, BROADCAST_CH, &bccb);
  unicast_open(&uc, UNICAST_CONTROL_CH, &uccb);

  cc2500_set_channel(10);

  while(1) {
    etimer_set(&transmission_et, TRANSMISSION_INTERVAL);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&transmission_et));

    if(rimeaddr_node_addr.u8[0] != 1) {
      rimeaddr_t dest;

      packetbuf_copyfrom(msg, 5);

      dest.u8[0] = 1;
      dest.u8[1] = 0;
/*      unicast_send(&uc, &dest);*/
      broadcast_send(&bc);
/*      leds_toggle(LEDS_RED);*/
    }
/*    if(msglen < MSGMAX) {*/
/*      msglen++;*/
/*    } else {*/
/*      msglen = MSGMIN;*/
/*    }*/
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
