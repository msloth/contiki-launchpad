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
 *         CC2500 header file: configuration
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#ifndef __CC2500_CONFIG_H__
#define __CC2500_CONFIG_H__

/* transmission power setting, 0..16 (17 elements) ranging from lowest to highest */
const uint8_t cc2500_txp[] = {0x50, 0x44, 0xC0, 0x84, 0x81, 0x46, 0x93, 0x55, 0x8D, 0xC6, 0x97, 0x6E, 0x7F, 0xA9, 0xBB, 0xFE, 0xFF};
#define CC2500_DEFAULT_TXPOWER    0xA9

/* the length of the below default settings = 2*the number of regs being set */
#define CC2500_DEF_CONF_LEN      (2 * 34)

const uint8_t cc2500_default_config[] = {
  /* tweaked SmartRF-settings: */
  // RX filterbandwidth = 540.000000 kHz
  // Deviation = 0.000000
  // Return state:  Return to RX state upon leaving either TX or RX
  // Datarate = 250 kbps
  // Modulation = (7) MSK
  // Manchester enable = (0) Manchester disabled
  // RF Frequency = 2433.000000 MHz
  // Channel spacing = 199.950000 kHz
  // Channel number = 0
  // Optimization = Sensitivity
  // Sync mode = (3) 30/32 sync word bits detected
  // Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
  // CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
  // Forward Error Correction = (0) FEC disabled
  // Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
  // Packetlength = 255
  // Preamble count = (2)  4 bytes
  // Append status = 1
  // Address check = (0) No address check
  // FIFO autoflush = 0
  // Device address = 0
  // GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
  // GDO2 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet

  CC2500_IOCFG2,   IOCFG_GDO_CFG_PKT_SYNCW_EOP,    // assert on sync word sent, de-assert on end-of-packet
  CC2500_IOCFG0,   IOCFG_GDO_CFG_PKT_SYNCW_EOP,    // assert on sync word sent, de-assert on end-of-packet
  CC2500_PKTLEN,   0xFF,
  CC2500_PKTCTRL1, 0x04,  // no address check or autoflush, do append status (RSSI/LQI and CRC ok)
  CC2500_PKTCTRL0, 0x05,  // no whitening, do calc and check CRC, length indicated by first byte

//  CC2500_ADDR,     0x00,
//  CC2500_CHANNR,   0x00,
  CC2500_FSCTRL1,  0x07,
  CC2500_FSCTRL0,  0x00,
  CC2500_FREQ2,    0x5D,

  CC2500_FREQ1,    0x93,
  CC2500_FREQ0,    0xB1,
  CC2500_MDMCFG4,  0x2D,
  CC2500_MDMCFG3,  0x3B,
  CC2500_MDMCFG2,  0x73,  // MSK, 30 of 32 bits

  CC2500_MDMCFG1,  0x22,  // 4 B pre-amble
  CC2500_MDMCFG0,  0xF8,
  CC2500_DEVIATN,  0x00,
  CC2500_MCSM1,    (3 << 4 | 0 << 2 | 0 << 0),  // Tx->idle, Rx->idle, CCA threshold
  CC2500_MCSM0,    0x08,  // no autocalibration
  CC2500_PKTLEN,   61,    // max packet len, to allow for length byte and footer; see errata 7

  CC2500_FOCCFG,   0x1D,
  CC2500_BSCFG,    0x1C,
  CC2500_AGCCTRL2, 0xC7,
  CC2500_AGCCTRL1, 0x00,
  CC2500_AGCCTRL0, 0xB2,

  CC2500_FREND1,   0xB6,
  CC2500_FREND0,   0x10,
  CC2500_FSCAL3,   0xEA,
  CC2500_FSCAL2,   0x0A,
  CC2500_FSCAL1,   0x00,

  CC2500_FSCAL0,   0x11,
  CC2500_FSTEST,   0x59,
  CC2500_TEST2,    0x88,
  CC2500_TEST1,    0x31,
  CC2500_TEST0,    0x0B,
};

#if 0
  CC2500_IOCFG2
  CC2500_IOCFG1
  CC2500_IOCFG0
  CC2500_FIFOTHR
  CC2500_SYNC1
  CC2500_SYNC0
  CC2500_PKTLEN
  CC2500_PKTCTRL1
  CC2500_PKTCTRL0
  CC2500_ADDR
  CC2500_CHANNR
  CC2500_FSCTRL1
  CC2500_FSCTRL0
  CC2500_FREQ2
  CC2500_FREQ1
  CC2500_FREQ0
  CC2500_MDMCFG4
  CC2500_MDMCFG3
  CC2500_MDMCFG2
  CC2500_MDMCFG1
  CC2500_MDMCFG0
  CC2500_DEVIATN
  CC2500_MCSM2
  CC2500_MCSM1
  CC2500_MCSM0
  CC2500_FOCCFG
  CC2500_BSCFG
  CC2500_AGCCTRL2
  CC2500_AGCCTRL1
  CC2500_AGCCTRL0
  CC2500_WOREVT1
  CC2500_WOREVT0
  CC2500_WORCTRL
  CC2500_FREND1
  CC2500_FREND0
  CC2500_FSCAL3
  CC2500_FSCAL2
  CC2500_FSCAL1
  CC2500_FSCAL0
  CC2500_RCCTRL1
  CC2500_RCCTRL0
  CC2500_FSTEST
  CC2500_PTEST
  CC2500_AGCTEST
  CC2500_TEST2
  CC2500_TEST1
  CC2500_TEST0
#endif

/* the radio is configured with: XXX No, changed...
      GFSK modulation
      X MHz freq
      variable packet len
      GDO0 interrupt at sync word received to EOP
      variable packet length
      CRC on, CRC autoflush off
      append two status bytes at end of packet, with RSSI, LQI and CRC ok
      30 of 32 status byte bits are ok
      4 byte preamble
      after tx go to idle
      after rx stay in rx
      only manual freq osc calibration
*/


#endif /* __CC2500_CONFIG_H__ */

