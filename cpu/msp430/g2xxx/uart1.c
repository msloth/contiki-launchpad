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
 * Machine dependent MSP430 UART1 code. Actually, right now, this is not using the UART1,
 but rather the UCA UART0
 */

#include "contiki.h"
#include "sys/energest.h"
#include "dev/uart1.h"
#include "dev/watchdog.h"
#include "sys/ctimer.h"
#include "lib/ringbuf.h"
#include "isr_compat.h"

static int (*uart1_input_handler)(unsigned char c);
static volatile uint8_t rx_in_progress = 0;
static volatile uint8_t transmitting = 0;

#if _MCU_ == 2553
#define   TXBUF        UCA0TXBUF
#define   RXBUF        UCA0RXBUF
#endif /* _MCU_ */


/*--------------------------------------------------------------------------*/
// shouldn't really be here (in uart1-putchar.c) but makes compiler warnings go away..
// why isn't the real one linked in?
#warning "Verify that int putchar(int c) works!"
int
putchar(int c)
/*putchar(unsigned char c)*/
{
  uart1_writeb(c);
  return c;
}

/*--------------------------------------------------------------------------*/
#ifdef UART1_CONF_TX_WITH_INTERRUPT
  #define TX_WITH_INTERRUPT UART1_CONF_TX_WITH_INTERRUPT
  #else /* UART1_CONF_TX_WITH_INTERRUPT */
  #define TX_WITH_INTERRUPT 1
#endif /* UART1_CONF_TX_WITH_INTERRUPT */

#if _MCU_ == 2553
#if TX_WITH_INTERRUPT
  #ifdef TX_INTERRUPT_BUFSIZE_CONF
  #define TXBUFSIZE TX_INTERRUPT_BUFSIZE_CONF
  #else
  #define TXBUFSIZE 128
  #endif
  static struct ringbuf txbuf;
  static uint8_t txbuf_data[TXBUFSIZE];
#endif /* TX_WITH_INTERRUPT */
#endif /* _MCU_ */

/*---------------------------------------------------------------------------*/
uint8_t
uart1_active(void)
{
#if _MCU_ == 2553
  return (UCA0STAT & UCBBUSY) | rx_in_progress | transmitting;
#else
  return 0;
#endif /* _MCU_ */
}
/*---------------------------------------------------------------------------*/
void
uart1_set_input(int (*input)(unsigned char c))
{
#if _MCU_ == 2553
  uart1_input_handler = input;
#endif /* _MCU_ */
}
/*---------------------------------------------------------------------------*/
void
uart1_writeb(unsigned char c)
{
#if _MCU_ == 2553
  watchdog_periodic();
#if TX_WITH_INTERRUPT
  /* Put the outgoing byte on the transmission buffer. If the buffer
     is full, we just keep on trying to put the byte into the buffer
     until it is possible to put it there. */
  while(ringbuf_put(&txbuf, c) == 0);

  /* If there is no transmission going, we need to start it by putting
     the first byte into the UART. */
  if(transmitting == 0) {
    transmitting = 1;
    /* Loop until the transmission buffer is available. */
    while((IFG2 & UTXIFG1) == 0);
    TXBUF = ringbuf_get(&txbuf);
  }

#else /* TX_WITH_INTERRUPT */
  while((IFG2 & UCA0TXIFG) == 0);
  TXBUF = c;
#endif /* TX_WITH_INTERRUPT */
#endif /* _MCU_ */
}
/*---------------------------------------------------------------------------*/
/**
 * Initalize the UART for serial communication with a host computer. It is
 * currently limited to 9600 baud due to Launchpad limitations.
 */
void
uart1_init(unsigned long ubr)
{
#if _MCU_ == 2553
  rx_in_progress = 0;
  transmitting = 0;

  /*  (TI) The recommended USCI initialization/re-configuration process is: */
  /*  1. Set UCSWRST (BIS.B #UCSWRST,&UCAxCTL1)*/
  UCA0CTL1 = UCSWRST;

  /*  2. Initialize all USCI registers with UCSWRST = 1 (including UCAxCTL1)*/
  UCA0CTL1 |= UCSSEL_2;   /* SMCLK, 4 MHz */
  ubr = 4000000ul/9600;
  UCA0BR0 = ubr & 0xff;
  UCA0BR1 = (ubr & 0xff00) >> 8;
  UCA0MCTL = UCBRS_1;

  /*  3. Configure ports.*/
  /* P12 for input (UART1RX), P11 for output (UART1TX) */
  P1SEL |= 1<<1 | 1<<2;
  P1SEL2 |= 1<<1 | 1<<2;

  /*  4. Clear UCSWRST via software (BIC.B #UCSWRST,&UCAxCTL1)*/
  UCA0CTL1 &= ~UCSWRST;

  /*  5. Enable interrupts (optional) via UCAxRXIE and/or UCAxTXIE*/
  /* Enable USART RX interrupt  */
  IFG2 &= ~UCA0RXIFG;
  IE2 |= UCA0RXIFG;

  #if TX_WITH_INTERRUPT
  ringbuf_init(&txbuf, txbuf_data, sizeof(txbuf_data));
  IE2 |= UCA0TXIFG;
  #endif /* TX_WITH_INTERRUPT */
#endif /* _MCU_ */
}
/*---------------------------------------------------------------------------*/
#if _MCU_ == 2553
ISR(USCIAB0RX, uartA0B0_rx_interrupt)
{
  uint8_t c;
  /* Check status register for receive errors. */
  if(UCA0STAT & UCRXERR) {
    c = UCA0RXBUF;   /* Clear error flags by forcing a dummy read. */
  } else {
    c = UCA0RXBUF;
    if(uart1_input_handler != NULL) {
      if(uart1_input_handler(c)) {
      	LPM4_EXIT;
      }
    }
  }
}
#endif /* _MCU_ */

/*---------------------------------------------------------------------------*/
#if _MCU_ == 2553
#if TX_WITH_INTERRUPT
ISR(USCIAB0TX, uartA0B0_tx_interrupt)
{
  if(IFG2 & UCA0TXIFG) {
    if(ringbuf_elements(&txbuf) == 0) {
      transmitting = 0;
    } else {
      TXBUF = ringbuf_get(&txbuf);
    }
  }
}
#endif /* TX_WITH_INTERRUPT */
#endif /* _MCU_ */
/*---------------------------------------------------------------------------*/









#if FROM_OLD_VERSION

#if F_CPU == 1000000ul
  /*
    clock: 1000000Hz
    desired baud rate: 115200bps
    division factor: 8.7
    effective baud rate: 114943bps
    maximum error: 0.6388us   7.36%
  */
  // XXX rather, use lower speed that has lower max error
  UBR00=0x08; UBR10=0x00; UMCTL0=0x5B; /* uart0 1000000Hz 114942bps */
  UBR01=0x08; UBR11=0x00; UMCTL1=0x5B; /* uart1 1000000Hz 114942bps */

#elif F_CPU == 4000000ul
  /*
    clock: 4000000Hz
    desired baud rate: 115200bps
    division factor: 34.7
    effective baud rate: 115274bps
    maximum error: 0.1249us   1.44%
  */
  UBR00=0x22; UBR10=0x00; UMCTL0=0xDD; /* uart0 4000000Hz 115273bps */
  UBR01=0x22; UBR11=0x00; UMCTL1=0xDD; /* uart1 4000000Hz 115273bps */

#elif F_CPU == 8000000ul
  /*
    clock: 8000000Hz
    desired baud rate: 115200bps
    division factor: 69.5
    effective baud rate: 115108bps
    maximum error: 0.0694us   0.80%
  */
  UBR00=0x45; UBR10=0x00; UMCTL0=0xAA; /* uart0 8000000Hz 115107bps */
  UBR01=0x45; UBR11=0x00; UMCTL1=0xAA; /* uart1 8000000Hz 115107bps */

#elif F_CPU == 12000000ul
  /*
    clock: 12000000Hz
    desired baud rate: 115200bps
    division factor: 104.1
    effective baud rate: 115274bps
    maximum error: 0.0555us   0.64%
  */
  UBR00=0x68; UBR10=0x00; UMCTL0=0x04; /* uart0 12000000Hz 115273bps */
  UBR01=0x68; UBR11=0x00; UMCTL1=0x04; /* uart1 12000000Hz 115273bps */

#elif F_CPU == 16000000ul
/*
  clock: 16000000Hz
  desired baud rate: 115200bps
  division factor: 138.9
  effective baud rate: 115191bps
  maximum error: 0.0277us   0.32%
*/
UBR00=0x8A; UBR10=0x00; UMCTL0=0xEF; /* uart0 16000000Hz 115190bps */
UBR01=0x8A; UBR11=0x00; UMCTL1=0xEF; /* uart1 16000000Hz 115190bps */
#else
#error CPU speed not defined, check settings in uart1.c!
#endif



#endif
