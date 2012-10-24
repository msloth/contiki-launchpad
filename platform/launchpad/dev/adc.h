/**
 * \addtogroup launchpad-platform
 *
 * @{
 */

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

/*
 * \file
 *         A simple ADC-implementation
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 *         
 */

#ifndef ADC_H
#define ADC_H

/* 
 * all the possible channels, you can use A0, A6 if resp. jumper disconnected
 * first and RXD/TXD if not used for printf/UART/USCI (jumpers as well).
 */
enum ADC_CHANNELS {
  A0 = 0,   // LED1
  A1 = 1,   // TXD
  A2 = 2,   // RXD
  A3 = 3,   // SW2
  A4 = 4,
  A5 = 5,
  A6 = 6,   // LED2
  A7 = 7,
  TEMP = 0xB,
};

#define ADC_NONBLOCK_GET(x)      adc_asynch_get(x)
#define ADC_BLOCKING_GET(x)      adc_synch_get(x)
#define ADC_CALLBACK_GET(x)      adc_irq_get(x)

process_event_t adc_event;

void adc_init(void);

/* 
 * use this when you plan on using the conversion a bit later (ca x ms later) 
 * and it is not that important. Will not block while converting.
 *    uint16_t val;
 *    adc_asynch_get(A7, &val);
 * 
 */
void adc_asynch_get(uint8_t adc_ch, uint16_t *val);

/*
 * use this when you need the result as soon as it is done; it will block until
 * it is finished with the conversion (<50 us)
 *    reading = adc_get_synch(A0);
 * 
 */
uint16_t adc_synch_get(uint8_t adc_ch);

/*
 * use this when you want to be notified as soon as the conversion is done.
 * The process will be sent an event when conversion is done.
 *    adc_irqevent_get(A0, PROCESS_CURRENT());
 *    PROCESS_WAIT_EVENT_UNTIL(ev == adc_event);
 * 
 */
void adc_irqevent_get(uint8_t adc_ch, struct process *p);

/*
 * use this when you want to be notified as soon as the conversion is done.
 * The process will be polled when conversion is done.
 *    adc_irqpoll_get(A7, PROCESS_CURRENT());
 * 
 */
void adc_irqpoll_get(uint8_t adc_ch, uint16_t *buf, struct process *p);

/* returns true if the ADC is currently doing a conversion */
uint8_t adc_busy(void);


#endif /* ADC_H */
/** @} */
