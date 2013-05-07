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

#else

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

#endif
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
  #else
  unicast_open(&uc, UNICAST_CONTROL_CH, &uccb);
  #endif

/*  cc2500_set_channel(10);*/

  if(rimeaddr_node_addr.u8[0] == 1) {
    while(1) {
      rimeaddr_t dest;
      dest.u8[0] = 2;
      dest.u8[1] = 0;
#if 0
      /* XXX with ContikiMAC, 45 bytes payload is the maximum size for unicasts, with 
          header is 64 bytes; with broadcast it's 47 as it doesn't use the 2-byte address */
      if(msglen < MSGMAX) {
        msglen++;
      } else {
        msglen = MSGMIN;
      }
#endif /* if 0; commented out code */
      packetbuf_copyfrom(msg, msglen);
      #if SEND_BC
      broadcast_send(&bc);
      #else
      unicast_send(&uc, &dest);
      #endif
      etimer_set(&transmission_et, TRANSMISSION_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&transmission_et));


#if 0
      dest.u8[0] = 3;
      dest.u8[1] = 0;
      packetbuf_copyfrom(msg, 10);
      unicast_send(&uc, &dest);
      etimer_set(&transmission_et, TRANSMISSION_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&transmission_et));
#endif /* if 0; commented out code */


#if 0
      cc2500_send(msg, 3);
      etimer_set(&transmission_et, TRANSMISSION_INTERVAL);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&transmission_et));
#endif /* if 0; commented out code */

    }
  } else {
    PROCESS_WAIT_EVENT_UNTIL(0);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#if NOWEDONTWANTTHISNO


--------------------------------------------------------------
compared with ContikiMAC
removed
    bursts
    phase optimizations
    no shortest packet len due to high margins (long guard times)
wake-up phase is not consistent, can be disturbed by transmissions

--------------------------------------------------------------
verify simpleRDC
    send broadcast
    send unicast
    receive broadcast
    duplicate packet rejection
    unicast ACK through SimpleRDC
    (unicast ACK through radio driver)

consider
    send even though RDC is off?
    send even though radio is off?
    send even though radio && RDC is off?
    any packetbuf attributes to take into account? reliable?
    mac callbacks, return codes
          
done
    idle listening through duty cycling
    verify timings:
        time for some packet lengths (small, medium, full)
        time to ACK

--------------------------------------------------------------
Debugging
  whats stopping transmissions from happening?
    SimpleRDC send_packet() stopping
      if(!simplerdc_is_on && !simplerdc_keep_radio_on) {
        --should not be there, we want to be able to send regardless of the RDC being on or not
      if(packetbuf_totlen() == 0) {
      if(packetbuf_hdralloc(sizeof(struct hdr)) == 0) {

      hdrlen = NETSTACK_FRAMER.create();
      if(hdrlen < 0) {



      --------------to here should be ok/reasonable
      receiving_packet depends on GDO-pin, check that the right one is used...

      if(NETSTACK_RADIO.receiving_packet() || (!is_broadcast && NETSTACK_RADIO.pending_packet())) {
            cc2500_receiving_packet(void)
            {
              /* if GDO is high, it means we are either receiving or sending a packet */
              if((CC2500_GDO_PORT(IN) & CC2500_GDO_PIN) && (CC2500_STATUS() != CC2500_STATE_TX)) {
                return 1;

            cc2500_pending_packet(void)
            {
              return pending_rxfifo;
      


      if(NETSTACK_RADIO.channel_clear() == 0) {
      ...then, it transmits    





mac_call_sent_callback(sent, ptr, MAC_TX_ERR_FATAL, 1);

Possible return values
  /**< The MAC layer transmission was OK. */
  MAC_TX_OK,

  /**< The MAC layer transmission could not be performed due to a collision. */
  MAC_TX_COLLISION,

  /**< The MAC layer did not get an acknowledgement for the packet. */
  MAC_TX_NOACK,

  /**< The MAC layer deferred the transmission for a later time. */
  MAC_TX_DEFERRED,

  /**< The MAC layer transmission could not be performed because of an
     error. The upper layer may try again later. */
  MAC_TX_ERR,

  /**< The MAC layer transmission could not be performed because of a
     fatal error. The upper layer does not need to try again, as the
     error will be fatal then as well. */
  MAC_TX_ERR_FATAL,








---------------------------------------------
QUESTIONS:
Radio driver:
    What states do the radio driver now put the radio in?
      after start/init
          --should be rx, power saving should turn off
      after packet rx
          --should be rx, power saving should turn off
      after packet tx
          --should be rx, power saving should turn off
    When is it calibrated?
      only on reset()  (?)
        periodic calibration for long running applications?
    how are these implemented?
      receiving_packet()
          if((CC2500_GDO_PORT(IN) & CC2500_GDO_PIN) && (CC2500_STATUS() != CC2500_STATE_TX)) {
      pending_packet()
          return pending_rxfifo;
      channel_clear()
          return cc2500_read_single(CC2500_PKTSTATUS) & PKTSTATUS_CCA;


SimpleRDC
  how is after tx wait for ACK implemented, what exits the tx?
      off() then return TX_OK;




packet address filtering and unicast ACKs---------------------------------------
  Radio HW does packet destination address filtering, we have to do ACK if unicast

  ACK unicasts in simpleRDC.
    --ACK in radio driver is only way to get fast response, in packet_read
      as otherwise process is polled, takes x ms.
        --must be able to decode receiver on radio layer
    --slow alternative: let simpleRDC do it, then slow as must wait for polled process
      -> must have long (10 ms) between tx wait time -> min 1/64 = 15 ms wake-up time

    ADDR is device address, here 1 byte
    PKTCTRL1 & 0x03 are the bits
        0 = no check
        1 = check, no broadcast
        2 = check, and broadcast = 0x00
        3 = check, and broadcast = 0x00 and 0xFF

    Radio frame/packet format
        preamble 4 bytes, sync 4 bytes, length 1 byte, address 1 byte, data n bytes, CRC-16 2 bytes

    to use automatic ACK:
        set
            CC2500_ADDR = rimeaddr_node_addr.u8[0]
            CC2500_PKTCTRL1 |= 2
        on tx: write len, write dest addr[0], then data
        on rx: read len, read dest addr, then process rest
               if to us (not broadcast) then ACK:
                   write len = 3
                         dest = u8[0]
                         dest2 = u8[1]
                   start sending




------------------------
  where does unicast ACK happen?
    --in cc2420 autoack only. If not activated, no ACK is sent and a tx takes entire period
    --remove header, use packetbuf attr ID instead  
        --no, PACKETBUF_ATTR_ID is not always sent with the packet, only those primitives that use the ID attr (runicast etc)

  why failed to parse error msg?
      --the contikimac header id was set and wrong, now we are using address only (not pseudo-version number)



-----------------------------------------------  

#if WITH_SIMPLERDC_HEADER
  struct hdr *chdr;

  hdrlen = packetbuf_totlen();
  if(packetbuf_hdralloc(sizeof(struct hdr)) == 0) {
    /* Failed to allocate space for simplerdc header */
    PRINTF("simplerdc: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
  chdr = packetbuf_hdrptr();
  chdr->id = simplerdc_ID;
  chdr->len = hdrlen;
  
  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("simplerdc: send failed, too large header\n");
    packetbuf_hdr_remove(sizeof(struct hdr));
    return MAC_TX_ERR_FATAL;
  }
  hdrlen += sizeof(struct hdr);
#else
  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("simplerdc: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
#endif










#if ENTIRE_NULLRDC_SEND
  int ret;
  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
  
  if(NETSTACK_FRAMER.create() >= 0) {
    ret = NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen());
    switch(ret) {
    case RADIO_TX_OK:
      ret = MAC_TX_OK;
      break;
    case RADIO_TX_COLLISION:
      ret = MAC_TX_COLLISION;
      break;
    case RADIO_TX_NOACK:
      ret = MAC_TX_NOACK;
      break;
    default:
      ret = MAC_TX_ERR;
      break;
    }

  } else {
    /* Failed to allocate space for headers */
    PRINTF("simplerdc: send failed, too large header\n");
    ret = MAC_TX_ERR_FATAL;
  }
  mac_call_sent_callback(sent, ptr, ret, 1);
#endif /* if 0; commented out code */





/*---------------------------------------------------------------------------*/
static int
OOOOLD__send_packet(mac_callback_t mac_callback, void *mac_callback_ptr, struct rdc_buf_list *buf_list)
{
  rtimer_clock_t t0;
  rtimer_clock_t encounter_time = 0;
  int strobes;
  uint8_t got_strobe_ack = 0;
  int hdrlen, len;
  uint8_t is_broadcast = 0;
  uint8_t is_reliable = 0;
  uint8_t is_known_receiver = 0;
  uint8_t collisions;
  int transmit_len;
  int ret;
  uint8_t simplerdc_was_on;
  uint8_t seqno;
#if WITH_simplerdc_HEADER
  struct hdr *chdr;
#endif /* WITH_simplerdc_HEADER */

 /* Exit if RDC and radio were explicitly turned off */
   if (!simplerdc_is_on && !simplerdc_keep_radio_on) {
    PRINTF("simplerdc: radio is turned off\n");
    return MAC_TX_ERR_FATAL;
  }
 
  if(packetbuf_totlen() == 0) {
    PRINTF("simplerdc: send_packet data len 0\n");
    return MAC_TX_ERR_FATAL;
  }

  packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
  if(rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER), &rimeaddr_null)) {
    is_broadcast = 1;
    PRINTF("simplerdc: send broadcast\n");

    if(broadcast_rate_drop()) {
      return MAC_TX_COLLISION;
    }
  } else {
#if UIP_CONF_IPV6
    PRINTF("simplerdc: send unicast to %02x%02x:%02x%02x:%02x%02x:%02x%02x\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[2],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[3],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[4],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[5],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[6],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[7]);
#else /* UIP_CONF_IPV6 */
    PRINTF("simplerdc: send unicast to %u.%u\n",
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0],
               packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[1]);
#endif /* UIP_CONF_IPV6 */
  }
  
  
  is_reliable = packetbuf_attr(PACKETBUF_ATTR_RELIABLE) || packetbuf_attr(PACKETBUF_ATTR_ERELIABLE);
  packetbuf_set_attr(PACKETBUF_ATTR_MAC_ACK, 1);



#if WITH_simplerdc_HEADER
  hdrlen = packetbuf_totlen();
  if(packetbuf_hdralloc(sizeof(struct hdr)) == 0) {
    /* Failed to allocate space for simplerdc header */
    PRINTF("simplerdc: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
  chdr = packetbuf_hdrptr();
  chdr->id = simplerdc_ID;
  chdr->len = hdrlen;
  
  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("simplerdc: send failed, too large header\n");
    packetbuf_hdr_remove(sizeof(struct hdr));
    return MAC_TX_ERR_FATAL;
  }
  hdrlen += sizeof(struct hdr);
#else
  /* Create the MAC header for the data packet. */
  hdrlen = NETSTACK_FRAMER.create();
  if(hdrlen < 0) {
    /* Failed to send */
    PRINTF("simplerdc: send failed, too large header\n");
    return MAC_TX_ERR_FATAL;
  }
#endif






  /* Make sure that the packet is longer or equal to the shortest
     packet length. */
  transmit_len = packetbuf_totlen();
  if(transmit_len < SHORTEST_PACKET_SIZE) {
    /* Pad with zeroes */
    uint8_t *ptr;
    ptr = packetbuf_dataptr();
    memset(ptr + packetbuf_datalen(), 0, SHORTEST_PACKET_SIZE - packetbuf_totlen());

    PRINTF("simplerdc: shorter than shortest (%d)\n", packetbuf_totlen());
    transmit_len = SHORTEST_PACKET_SIZE;
  }




  packetbuf_compact();
  NETSTACK_RADIO.prepare(packetbuf_hdrptr(), transmit_len);




  /* Remove the MAC-layer header since it will be recreated next time around. */
  packetbuf_hdr_remove(hdrlen);







  /* By setting we_are_sending to one, we ensure that the rtimer
     powercycle interrupt do not interfere with us sending the packet. */
  we_are_sending = 1;

  /* If we have a pending packet in the radio, we should not send now,
     because we will trash the received packet. Instead, we signal
     that we have a collision, which lets the packet be received. This
     packet will be retransmitted later by the MAC protocol
     instread. */
  if(NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet()) {
    we_are_sending = 0;
    PRINTF("simplerdc: collision receiving %d, pending %d\n", NETSTACK_RADIO.receiving_packet(), NETSTACK_RADIO.pending_packet());
    return MAC_TX_COLLISION;
  }
  
  /* Switch off the radio to ensure that we didn't start sending while
     the radio was doing a channel check. */
  off();

  /* Send a train of strobes until the receiver answers with an ACK. */
  strobes = 0;
  collisions = 0;
  got_strobe_ack = 0;

  /* Set simplerdc_is_on to one to allow the on() and off() functions
     to control the radio. We restore the old value of
     simplerdc_is_on when we are done. */
  simplerdc_was_on = simplerdc_is_on;
  simplerdc_is_on = 1;

#if !RDC_CONF_HARDWARE_CSMA
  /* Check if there are any transmissions by others. */
  if(is_receiver_awake == 0) {
	  int i;
    for(i = 0; i < CCA_COUNT_MAX_TX; ++i) {
      t0 = RTIMER_NOW();
      on();
#if CCA_CHECK_TIME > 0
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_CHECK_TIME)) { }
#endif
      if(NETSTACK_RADIO.channel_clear() == 0) {
        collisions++;
        off();
        break;
      }
      off();
      t0 = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + CCA_SLEEP_TIME)) { }
    }
  }

  if(collisions > 0) {
    we_are_sending = 0;
    off();
    PRINTF("simplerdc: collisions before sending\n");
    simplerdc_is_on = simplerdc_was_on;
    return MAC_TX_COLLISION;
  }
#endif /* RDC_CONF_HARDWARE_CSMA */

#if !RDC_CONF_HARDWARE_ACK
  if(!is_broadcast) {
  /* Turn radio on to receive expected unicast ack.
      Not necessary with hardware ack detection, and may trigger an unnecessary cca or rx cycle */	 
     on();
  }
#endif

  watchdog_periodic();
  t0 = RTIMER_NOW();
  seqno = packetbuf_attr(PACKETBUF_ATTR_MAC_SEQNO);
  for(strobes = 0, collisions = 0;
      got_strobe_ack == 0 && collisions == 0 &&
      RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + STROBE_TIME); strobes++) {

    watchdog_periodic();

    if((is_receiver_awake || is_known_receiver) && !RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + MAX_PHASE_STROBE_TIME)) {
      PRINTF("miss to %d\n", packetbuf_addr(PACKETBUF_ADDR_RECEIVER)->u8[0]);
      break;
    }

    len = 0;

    
    {
      rtimer_clock_t wt;
      rtimer_clock_t txtime;
      int ret;

      txtime = RTIMER_NOW();
      ret = NETSTACK_RADIO.transmit(transmit_len);

#if RDC_CONF_HARDWARE_ACK
     /* For radios that block in the transmit routine and detect the ACK in hardware */
      if(ret == RADIO_TX_OK) {
        if(!is_broadcast) {
          got_strobe_ack = 1;
          encounter_time = txtime;
          break;
        }
      } else if (ret == RADIO_TX_NOACK) {
      } else if (ret == RADIO_TX_COLLISION) {
          PRINTF("simplerdc: collisions while sending\n");
          collisions++;
      }
	  wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + INTER_PACKET_INTERVAL)) { }
#else
     /* Wait for the ACK packet */
      wt = RTIMER_NOW();
      while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + INTER_PACKET_INTERVAL)) { }

      if(!is_broadcast && (NETSTACK_RADIO.receiving_packet() ||
                           NETSTACK_RADIO.pending_packet() ||
                           NETSTACK_RADIO.channel_clear() == 0)) {
        uint8_t ackbuf[ACK_LEN];
        wt = RTIMER_NOW();
        while(RTIMER_CLOCK_LT(RTIMER_NOW(), wt + AFTER_ACK_DETECTECT_WAIT_TIME)) { }

        len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
        if(len == ACK_LEN && seqno == ackbuf[ACK_LEN-1]) {
          got_strobe_ack = 1;
          encounter_time = txtime;
          break;
        } else {
          PRINTF("simplerdc: collisions while sending\n");
          collisions++;
        }
      }
#endif /* RDC_CONF_HARDWARE_ACK */
    }
  }

  off();

  PRINTF("simplerdc: send (strobes=%u, len=%u, %s, %s), done\n", strobes,
         packetbuf_totlen(),
         got_strobe_ack ? "ack" : "no ack",
         collisions ? "collision" : "no collision");

  simplerdc_is_on = simplerdc_was_on;
  we_are_sending = 0;

  /* Determine the return value that we will return from the
     function. We must pass this value to the phase module before we
     return from the function.  */
  if(collisions > 0) {
    ret = MAC_TX_COLLISION;
  } else if(!is_broadcast && !got_strobe_ack) {
    ret = MAC_TX_NOACK;
  } else {
    ret = MAC_TX_OK;
  }

  return ret;
}

#endif /* if 0; commented out code */
