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
 *         Driver for HPDL-1414 "Four character Smart Alphanumeric Displays"
 *         from Avago. Parallell interface, decoding ASCII and shows on the
 *         beautiful 2.85 mm display:
 *         http://www.decadecounter.com/vta/articleview.php?item=465
 *
 *         Top view:
 *            1.6   2.3   2.2   2.1   2.0    GND        LP/MSP430G2553 pin
 *            12    11    10    9     8      7          pin# acc to datasheet
 *            |     |     |     |     |      |
 *         +--|-----|-----|-----|-----|------|----+
 *         |  D6    D3    D2    D1    D0    GND   |     pin function
 *         |                                      |
 *         |                                      |
 *         |    (1)     (2)       (3)     (4)     |     char# acc to this driver
 *         |                                      |
 *         |                                      |
 *         |  D5    D4    WR    A1    A0    Vcc   |     pin function
 *         +--|-----|-----|-----|-----|------|----+
 *            |     |     |     |     |      |
 *            1     2     3     4     5      6          pin# acc to datasheet
 *            2.5   2.4   1.7   1.5   1.4    +5V        LP/MSP430G2553 pin
 *
 *         HP-versions are chafered in pin1-corner
 *         Siemens-versions (?) have a small letter on the side of pin1, and a
 *            big via in the opposite corner, by GND.
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#ifndef __HPDL_DRIVER_H__
#define __HPDL_DRIVER_H__

/*---------------------------------------------------------------------------*/
/*
 * This driver for HPDL-1414 assumes the display is connected as follows
 *    Vcc to HDPL is 5V, this can be found close to the USB-connector on LP
 *          (marked TP1 or TP3, check with voltmeter)
 *    GND to ordinary LP GND
 *    A0 to P1.4, A1 to P1.5
 *    D0 to P2.0, D1 to P2.1, ..., D5 to P2.5
 *    D6 to P1.7 (not enough pins on port 2 on '2452/'2553)
 *    WR to P1.6 so remove jumper for LED and don't use leds_x() with that LED (green?)
 */



/* define pins for digit address */
#define HPDL_ADDRESS_PORT(type)         P1##type
#define HPDL_A0_PIN                     (1 << 4)
#define HPDL_A1_PIN                     (1 << 5)

/* define pin for WR (active low) */
#define HPDL_WR_PORT(type)              P1##type
#define HPDL_WR_PIN                     (1 << 7)
#warning "HPDL uses the same pin as LED2 so if HPDL is used, make sure to not use leds_x() and remove the jumper on the board."

/*
 * define the pins for the digits parallell interface.
 * assumes the whole port 2 is used for digits (needs 7 pins) but still lacks
 * one pin as port 2 on msp430g2452/2553 only has 6 pins on port 2.
 *
 *
 * assumes pin 0 == D0, pin 1 == D1 etc.
 *
 *
 *
 *
 */
#define HPDL_DIGIT_PORT(type)           P2##type
#define HPDL_DIGIT_PINS                 (0x3f)
#define HPDL_DIGIT_EP_PORT(type)        P1##type
#define HPDL_DIGIT_EP_PIN               (1 << 6)

/* use scrolling; set to zero to save some space if not used */
#define HPDL_USE_SCROLL                 0
/*--------------------------------------------------------------------------*/
void hpdl_init(void);

void hpdl_write_char(uint8_t pos, uint8_t ch);

void hpdl_write_string(char *s);

#if HPDL_USE_SCROLL
void hpdl_scroll_string(char *s, clock_time_t intvl);

void hpdl_scroll_stop(void);
#endif    /* HPDL_USE_SCROLL */

void hpdl_clear(void);
/*--------------------------------------------------------------------------*/
#endif /* __HPDL_DRIVER_H__ */

