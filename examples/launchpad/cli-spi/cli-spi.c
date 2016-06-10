/*
 * Copyright (c) 2016, Marcus Linderoth
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
 *        Simple SPI command-tool, similar to Buspirate
 *        usage
 *          compile, upload to LP, login over serial
 *          remember to adjust the rxtx headers on the board if necessary
 *          enter command in hex, no prolog of eg 0x or h or similar
 *          the device will push over SPI and return what the other device answered on SPI
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/serial-line.h"
/*---------------------------------------------------------------------------*/
PROCESS(serial_read_process, "CLI-SPI");
AUTOSTART_PROCESSES(&serial_read_process);
/*---------------------------------------------------------------------------*/
/* seems like the UART buffer is max 15 bytes anyway */
#define SPIBUFMAX 16
static uint8_t spi_buf[SPIBUFMAX];
static int clock_idle_low = 1;

#define SPI_CLK_PORT  P2OUT
#define SPI_CLK_DIRP  P2DIR
#define SPI_CLK_BIT   (1 << 0)

#define SPI_MOSI_PORT P2OUT
#define SPI_MOSI_DIRP P2DIR
#define SPI_MOSI_BIT  (1 << 1)

#define SPI_MISO_PXIN P2IN
#define SPI_MISO_DIRP P2DIR
#define SPI_MISO_BIT  (1 << 2)

#define SPI_CS_PORT   P2OUT
#define SPI_CS_DIRP   P2DIR
#define SPI_CS_BIT    (1 << 3)

#define GPIO_SET(port, bit) (port |= bit)
#define GPIO_RESET(port, bit) (port &= ~bit)
#define GPIO_GET(port, bit) (port & bit)
/*---------------------------------------------------------------------------*/
/* simplistic bit-banged SPI of one byte */
static uint8_t
rwbyte(uint8_t output)
{
  int i;
  uint8_t input = 0;

  for(i = 0; i < 8; i++) {
    /* Write data on MOSI pin */
    if(output & 0x80) {
      GPIO_SET(SPI_MOSI_PORT, SPI_MOSI_BIT);
    } else {
      GPIO_RESET(SPI_MOSI_PORT, SPI_MOSI_BIT);
    }
    output <<= 1;

    /* flip clock */
    if(clock_idle_low) {
      GPIO_SET(SPI_CLK_PORT, SPI_CLK_BIT);
    } else {
      GPIO_RESET(SPI_CLK_PORT, SPI_CLK_BIT);
    }

    /* Read data from MISO pin */
    input <<= 1;
    if(GPIO_GET(SPI_MISO_PXIN, SPI_MISO_BIT) != 0) {
      input |= 0x1;
    }

    /* flip clock */
    if(clock_idle_low) {
      GPIO_RESET(SPI_CLK_PORT, SPI_CLK_BIT);
    } else {
      GPIO_SET(SPI_CLK_PORT, SPI_CLK_BIT);
    }
  }
  return input;
}
/*---------------------------------------------------------------------------*/
static void
spi_inout(uint8_t len)
{
  int i;
  uint8_t temp;

  if(len > 0) {
    GPIO_RESET(SPI_CS_PORT, SPI_CS_BIT);
    for(i = 0; i < len; i++) {
      temp = spi_buf[i];
      spi_buf[i] = rwbyte(temp);
    }
    GPIO_SET(SPI_CS_PORT, SPI_CS_BIT);
  }
}
/*--------------------------------------------------------------------------*/
static void
spi_init(void)
{
  /* Set pins to output/input mode */
  SPI_CLK_DIRP |= SPI_CLK_BIT;
  SPI_CS_DIRP |= SPI_CS_BIT;
  SPI_MOSI_DIRP |= SPI_MOSI_BIT;
  SPI_MISO_DIRP &= ~SPI_MISO_BIT;

  /* set CS and CLK defaults */
  GPIO_SET(SPI_CS_PORT, SPI_CS_BIT);
  GPIO_RESET(SPI_CLK_PORT, SPI_CLK_BIT);
}
/*--------------------------------------------------------------------------*/
static int
parsehexdigit(char c)
{
  if(c >= '0' && c <= '9') {
    return c - '0';
  }
  if(c >= 'a' && c <= 'f') {
    return c - 'a' + 10;
  }
  if(c >= 'A' && c <= 'F') {
    return c - 'A' + 10;
  }
  return 0;
}
/*--------------------------------------------------------------------------*/
static void
print_help(void)
{
  printf("commands:\n");
  printf("  h for help\n");
  printf("  enter bytes to send in hex, without 0x or h, eg\n");
  printf("  the corresponding read bytes will be printed out\n");
  printf("    018a0113\n");
  printf("  to eg send 2B and read out 3 more, just type zeroes, eg\n");
  printf("    018a000000\n");
  printf("  configure clock idle low\n  scl\n");
  printf("  configure clock idle high\n  sch\n");

  printf("current config:\n");
  printf("SCLK p%02x, pin%02x\n", SPI_CLK_PORT, SPI_CLK_BIT);
  printf("MOSI p%02x, pin%02x\n", SPI_MOSI_PORT, SPI_MOSI_BIT);
  printf("MISO p%02x, pin%02x\n", SPI_MISO_PORT, SPI_MISO_BIT);
  printf("CS p%02x, pin%02x\n", SPI_CS_PORT, SPI_CS_BIT);
}
/*--------------------------------------------------------------------------*/
/*
 * wait for serial input, parse, act accordingly, loop
 */
PROCESS_THREAD(serial_read_process, ev, data)
{
  PROCESS_BEGIN();

  printf("starting CLI-SPI\n");
  print_help();

  spi_init();
  while (1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);

    /* parse UART data to hex buffer and print it out, for lack of RAM, we reuse the same buffer */
    uint8_t *datab = data;
    if(datab[0] == 's') {
      if(datab[1] == 'c') {
        if(datab[2] == 'l') {
          clock_idle_low = 1;
          GPIO_RESET(SPI_CLK_PORT, SPI_CLK_BIT);
        } if(datab[2] == 'h') {
          clock_idle_low = 0;
          GPIO_SET(SPI_CLK_PORT, SPI_CLK_BIT);
        }
      }
      
    } else {
      int len = strlen(datab);
      // printf("Got %d: %s\n", len, datab);
      if(len / 2 <= SPIBUFMAX) {
        int i;
        for(i = 0; i < len/2; i++) {
          uint8_t temp = 0;
          temp = parsehexdigit(datab[i * 2]) << 4;
          temp |= parsehexdigit(datab[i * 2 + 1]);
          spi_buf[i] = temp;
        }
    
        /* perform SPI transaction */
        spi_inout(len/2);

        /* print the resulting input buffer */
        printf("\nReturned: ");
        for(i = 0; i < len/2; i++) {
          printf("%02x", spi_buf[i]);
        }
        printf(".\n");
      }
    } else {
      /* too large input */
    }
  }
  PROCESS_END();
}
/*--------------------------------------------------------------------------*/
