/**
 * \defgroup launchpad-platform TI MSP430 Launchpad Contiki port.
 *
 * TI MSP430 Launchpad Contiki port.
 *
 * @{
 */

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
 *         Contiki configuration file for Launchpad platform
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#ifndef CONTIKI_CONF_H
#define CONTIKI_CONF_H

#ifdef PLATFORM_CONF_H
#include PLATFORM_CONF_H
#else
#include "platform-conf.h"
#endif /* PLATFORM_CONF_H */
/*---------------------------------------------------------------------------*/
/* we are not using uip6, due to memory constraints. Needs to be set early. */
#define WITH_UIP6                     0

/* include the project config */
/* PROJECT_CONF_H might be defined in the project Makefile */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */

/* These can be lowered to reduce RAM size, to a certain point. Remember that
  a printf may not exceed this buffer size or unpredictable things will happen! */
// dev/serial-line.h
#define SERIAL_LINE_CONF_BUFSIZE            32

// g2xxxx/uart1.c; XXX until interrupt driven UART is done, these have no effect (and no RAM will be allocated either)
#define TX_INTERRUPT_BUFSIZE_CONF           32
#define RX_INTERRUPT_BUFSIZE_CONF           32
/*--------------------------------------------------------------------------*/
/* define the network and some related settings ------------------ */
#ifndef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     nullmac_driver
#endif /* NETSTACK_CONF_MAC */

#ifndef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     simplerdc_driver
//#define NETSTACK_CONF_RDC     nullrdc_driver
#endif /* NETSTACK_CONF_RDC */

#ifndef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_nullmac
#endif /* NETSTACK_CONF_FRAMER */

#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO   cc2500_driver
#endif /* NETSTACK_CONF_RADIO */

/* set packetbuf buffer sizes */
#define PACKETBUF_CONF_SIZE         64    /* 128 */
//#define PACKETBUF_CONF_SIZE         66
#define PACKETBUF_CONF_HDR_SIZE     16    /* 48 */  // XXX how do they relate?

/* what channel to start at */
#ifndef RF_CHANNEL
#define RF_CHANNEL                  1
#endif /* RF_CHANNEL */





/* make sure to use short Rime addresses */
#define RIMEADDR_CONF_SIZE          2

/* save ~80 B RAM by not using Announcements*/
#define USE_ANNOUNCEMENTS_CONF      0


/* used by eg ContikiMAC; for future use */
#ifndef NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE 8
#endif /* NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE */






#if !WITH_UIP6
  /* Network setup for non-IPv6 (rime). */
  #define NETSTACK_CONF_NETWORK            rime_driver

  #define COLLECT_CONF_ANNOUNCEMENTS       0
  #define CXMAC_CONF_ANNOUNCEMENTS         0
  #define XMAC_CONF_ANNOUNCEMENTS          0
  #define CONTIKIMAC_CONF_ANNOUNCEMENTS    0

  #ifndef COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS
  #define COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS     0
  #endif /* COLLECT_NEIGHBOR_CONF_MAX_COLLECT_NEIGHBORS */

  /* remove queuebufs */
  #define QUEUEBUF_CONF_NUM           0
  #define QUEUEBUF_CONF_STATS         0
  #define QUEUEBUF_CONF_REF_NUM       0

  #ifndef TIMESYNCH_CONF_ENABLED
  #define TIMESYNCH_CONF_ENABLED           0
  #endif /* TIMESYNCH_CONF_ENABLED */

  #if TIMESYNCH_CONF_ENABLED
  /* CC2420 SDF timestamps must be on if timesynch is enabled. */
  #undef CC2420_CONF_SFD_TIMESTAMPS
  #define CC2420_CONF_SFD_TIMESTAMPS       0
  #endif /* TIMESYNCH_CONF_ENABLED */

#endif /* !WITH_UIP6 */

/* Specify whether the RDC layer should enable
   per-packet power profiling. */
#define CONTIKIMAC_CONF_COMPOWER                0
#define XMAC_CONF_COMPOWER                      0
#define CXMAC_CONF_COMPOWER                     0
#define CONTIKIMAC_CONF_BROADCAST_RATE_LIMIT    0

#define IEEE802154_CONF_PANID                   0xABCD


/*--------------------------------------------------------------------------*/
/* not so important configs for us, we are not using these features but save for
future use, compiler errors, and completeness */

#define PACKETBUF_CONF_ATTRS_INLINE     1     /* less: 72B ROM, 0 RAM if set to 1; what does it do? */

#define PROFILE_CONF_ON                 0
#define ENERGEST_CONF_ON                0

#define PROCESS_CONF_NUMEVENTS          8     /* each takes 8 B RAM */
#define PROCESS_CONF_STATS              1     /* less: 18 B ROM, 0 RAM if set to 1 */
//#define PROCESS_CONF_FASTPOLL         4


//#define AODV_COMPLIANCE
//#define AODV_NUM_RT_ENTRIES           32
//#define WITH_ASCII                    1
//#define ELFLOADER_CONF_TEXT_IN_ROM    0
//#define SHELL_VARS_CONF_RAM_BEGIN     0x1100
//#define SHELL_VARS_CONF_RAM_END       0x2000




#if WITH_UIP6
  /* Network setup for IPv6 */
  #define NETSTACK_CONF_NETWORK sicslowpan_driver

  /* Specify a minimum packet size for 6lowpan compression to be
     enabled. This is needed for ContikiMAC, which needs packets to be
     larger than a specified size, if no ContikiMAC header should be
     used. */
  #define SICSLOWPAN_CONF_COMPRESSION_THRESHOLD 63
  #define CONTIKIMAC_CONF_WITH_CONTIKIMAC_HEADER 0

  #define CXMAC_CONF_ANNOUNCEMENTS         0

  #ifndef QUEUEBUF_CONF_NUM
  #define QUEUEBUF_CONF_NUM                8
  #endif


  #define UIP_CONF_ICMP_DEST_UNREACH 1
  #define UIP_CONF_DHCP_LIGHT
  #define UIP_CONF_LLH_LEN         0
  #ifndef  UIP_CONF_RECEIVE_WINDOW
  #define UIP_CONF_RECEIVE_WINDOW  48
  #endif
  #ifndef  UIP_CONF_TCP_MSS
  #define UIP_CONF_TCP_MSS         48
  #endif
  #define UIP_CONF_MAX_CONNECTIONS 4
  #define UIP_CONF_MAX_LISTENPORTS 8
  #define UIP_CONF_UDP_CONNS       12
  #define UIP_CONF_FWCACHE_SIZE    30
  #define UIP_CONF_BROADCAST       1
  #define UIP_ARCH_IPCHKSUM        1
  #define UIP_CONF_UDP             1
  #define UIP_CONF_UDP_CHECKSUMS   1
  #define UIP_CONF_PINGADDRCONF    0
  #define UIP_CONF_LOGGING         0
  #define UIP_CONF_TCP_SPLIT       0
  #define UIP_CONF_ICMP_DEST_UNREACH 1
  #define UIP_CONF_DHCP_LIGHT
  #define UIP_CONF_LLH_LEN         0
  #ifndef  UIP_CONF_RECEIVE_WINDOW
  #define UIP_CONF_RECEIVE_WINDOW  48
  #endif
  #ifndef  UIP_CONF_TCP_MSS
  #define UIP_CONF_TCP_MSS         48
  #endif
  #define UIP_CONF_MAX_CONNECTIONS 4
  #define UIP_CONF_MAX_LISTENPORTS 8
  #define UIP_CONF_UDP_CONNS       12
  #define UIP_CONF_FWCACHE_SIZE    30
  #define UIP_CONF_BROADCAST       1
  #define UIP_ARCH_IPCHKSUM        1
  #define UIP_CONF_UDP             1
  #define UIP_CONF_UDP_CHECKSUMS   1
  #define UIP_CONF_PINGADDRCONF    0
  #define UIP_CONF_LOGGING         0
  #define UIP_CONF_TCP_SPLIT       0


#endif /* WITH_UIP6 */

#endif /* CONTIKI_CONF_H */
/** @} */
