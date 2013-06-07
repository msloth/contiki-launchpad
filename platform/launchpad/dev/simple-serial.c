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

/*---------------------------------------------------------------------------*/
#include "simple-serial.h"
#include "uart0.h"
#include "watchdog.h"

static void printchar(char c);

#if _MCU_ == 2553
#define   TXBUF        UCA0TXBUF
#endif /* _MCU_ */

static const char hlut[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
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
    /* simple-serial is output only, don't do anything with the input */
  }
}
#endif /* _MCU_ */
/*---------------------------------------------------------------------------*/
void
simple_serial_init(void)
{

  /* taken from the UART0-drivers */
#if _MCU_ == 2553
  uint16_t ubr;
  /*  (TI) The recommended USCI initialization/re-configuration process is: */
  /*  1. Set UCSWRST (BIS.B #UCSWRST,&UCAxCTL1)*/
  UCA0CTL1 = UCSWRST;

  /*  2. Initialize all USCI registers with UCSWRST = 1 (including UCAxCTL1)*/
  UCA0CTL1 |= UCSSEL_2;   /* SMCLK, 4 MHz */
  ubr = (4000000ul / 9600);
  UCA0BR0 = ubr & 0xff;
  UCA0BR1 = (ubr & 0xff00) >> 8;
  UCA0MCTL = UCBRS_1;

  /*  3. Configure ports.*/
  /* P1.2 for input (UART0RX), P1.1 for output (UART0TX) */
  P1SEL  |= 1<<1 | 1<<2;
  P1SEL2 |= 1<<1 | 1<<2;

  /*  4. Clear UCSWRST via software (BIC.B #UCSWRST,&UCAxCTL1)*/
  UCA0CTL1 &= ~UCSWRST;

  /*  5. Enable interrupts (optional) via UCAxRXIE and/or UCAxTXIE*/
  /* Enable USART RX interrupt  */
  IFG2 &= ~UCA0RXIFG;
  IE2 |= UCA0RXIFG;
#endif /* _MCU_ */
}
/*---------------------------------------------------------------------------*/
void
simple_serial_print_str(char *s)
{
  char c = 'A';
  while(c != 0) {
    c = *s++;
    if(c == '\\' && *s == 'n') {
      /* print line return and continue */
      printchar('\n');
      s++;
    } else if(c == '\\' && *s == 't') {
      printchar('\t');
      s++;
    } else {
      printchar(c);
    }
  }
}
/*---------------------------------------------------------------------------*/
/* print a string and an int */
void
simple_serial_print_strint(char *s, uint16_t i)
{
  uint8_t intprinted = 0;
  char c = 'A';
  while(c != 0) {
    c = *s++;
    if(c == '\\' && *s == 'n') {
      /* print line return and continue */
      printchar('\n');
      s++;
    } else if(c == '\\' && *s == 't') {
      printchar('\t');
      s++;
    } else if(c == '%' && *s == 'u') {
      intprinted = 1;
      simple_serial_print_u16(i);
    } else {
      printchar(c);
    }
  }

  if(intprinted == 0) {
    simple_serial_print_u16(i);
  }
  printchar('\n');
}
/*---------------------------------------------------------------------------*/
void
simple_serial_print_hex_u8(uint8_t hu)
{
  printchar(hlut[(hu & 0xf0) >> 8]);
  printchar(hlut[hu & 0x0f]);
}
/*---------------------------------------------------------------------------*/
void
simple_serial_print_hex_u16(uint16_t hu)
{
  simple_serial_print_hex_u8((uint8_t) (hu >> 12));
  simple_serial_print_hex_u8((uint8_t) (hu >> 8));
  simple_serial_print_hex_u8((uint8_t) (hu >> 4));
  simple_serial_print_hex_u8((uint8_t) (hu));
}
/*---------------------------------------------------------------------------*/
void
simple_serial_print_hex_u32(uint32_t hu)
{
  simple_serial_print_hex_u8((uint8_t) (hu >> 28));
  simple_serial_print_hex_u8((uint8_t) (hu >> 24));
  simple_serial_print_hex_u8((uint8_t) (hu >> 20));
  simple_serial_print_hex_u8((uint8_t) (hu >> 16));
  simple_serial_print_hex_u8((uint8_t) (hu >> 12));
  simple_serial_print_hex_u8((uint8_t) (hu >> 8));
  simple_serial_print_hex_u8((uint8_t) (hu >> 4));
  simple_serial_print_hex_u8((uint8_t) (hu));
}
/*---------------------------------------------------------------------------*/
void
simple_serial_print_u8(uint8_t u)
{
  uint8_t t = u;
  uint8_t i = 0;
  while(t > 100) {
    i++;
    t -= 100;
  }
  if(i > 0) {
    printchar('0' + i);
  }

  i = 0;
  while(t > 10) {
    i++;
    t -= 10;
  }
  if(i > 0) {
    printchar('0' + i);
  }
  printchar('0' + t);
}
/*---------------------------------------------------------------------------*/
void
simple_serial_print_u16(uint16_t u)
{
  uint16_t t = u;
  uint8_t i = 0;
  uint8_t buf[] = {'0', '0', '0', '0', '0'};

  if(u == 0) {
    printchar('0');
    printchar('0');
    printchar('0');
    printchar('0');
    printchar('0');
    return;
  }



  i = 0;
  while(t >= 10000) {
    i++;
    t -= 10000;
  }
  if(i > 0) {
    buf[4] = '0' + i;
  }



  i = 0;
  while(t >= 1000) {
    i++;
    t -= 1000;
  }
  if(i > 0) {
    buf[3] = '0' + i;
  }



  i = 0;
  while(t >= 100) {
    i++;
    t -= 100;
  }
  if(i > 0) {
    buf[2] = '0' + i;
  }




  i = 0;
  while(t >= 10) {
    i++;
    t -= 10;
  }
  if(i > 0) {
    buf[1] = '0' + i;
  }

  buf[0] = '0' + t;
  for(i = 0; i < 5; i += 1) {
    printchar(buf[4-i]);
  }
}
/*---------------------------------------------------------------------------*/
void
simple_serial_print_u32(uint32_t u)
{
  /* not yet implemented. */
}
/*---------------------------------------------------------------------------*/
void
simple_serial_print_raw(uint8_t *u, uint8_t len)
{
  uint8_t i;
  uint8_t *p = u;

  for(i = 0; i < len; i++) {
    printchar(*p++);
  }
}
/*---------------------------------------------------------------------------*/
static void
printchar(char c)
{
#if _MCU_ == 2553
  /* wait until ready and then just put the byte on the tx buffer */
  watchdog_periodic();
  while((IFG2 & UCA0TXIFG) == 0);
  TXBUF = c;
#endif /* _MCU_ */
}
/*---------------------------------------------------------------------------*/