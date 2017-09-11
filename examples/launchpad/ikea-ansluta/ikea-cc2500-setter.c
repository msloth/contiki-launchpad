#include <stdlib.h>
#include <stdio.h>
#include "contiki.h"
#include "dev/spi.h"
#include "dev/cc2500-arch.h"
#include "dev/cc2500-const.h"
#include "watchdog.h"
/*---------------------------------------------------------------------------*/
/*
 * This application toggles an IKEA Ansluta remote control LED light. The Ansluta
 * uses a cc2500 radio. The remote control radio usage was sniffed with a logic
 * analyzer which is mirrored here. The Ansluta can only switch between 0, 35, 100%
 * dim levels, which is set using a specific byte in the packet.
 * The logic sniff was performed on a remote that was already paired, so the
 * pairing information is included in the packet, but unclear now exactly what
 * bytes is the pairing info.
 * 
 * todo and notes:
 * set PATABLE to 0xff for highest transmission power setting
 * SCLK was inverted against the capture
 *     -- no, mine is right, it should be SCLK initially low, but the IKEA one
 *     is initial high, which is against the datasheet. MOSI is latched on CLK
 *     low->high transition.
 *     IKEA: initial low, set MOSI at clk high->low, flip CLK low->high in the middle
 */
/*---------------------------------------------------------------------------*/
static void
strobe(uint8_t s)
{
  cc2500_strobe(s);
}
/*---------------------------------------------------------------------------*/
static uint8_t
status(void)
{
  return cc2500_strobe(CC2500_SNOP);
}
/*---------------------------------------------------------------------------*/
static void
write_burst(uint8_t *src, uint8_t len)
{
  // uint8_t s;
  int i;
  /* sth (the button?) sets the MISO pin so lets reset all pins we need */
  SPI_PORT(SEL)  |= SPI_MISO | SPI_MOSI | SPI_SCL;
  SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  CC2500_SPI_ENABLE();
  SPI_WRITE(src[0]);
  // s = SPI_RXBUF;
  for(i = 1; i < len; i += 1) {
    SPI_WRITE_FAST(src[i]);
  }
  SPI_WAIT_WHILE_BUSY();
  CC2500_SPI_DISABLE();
}
/*---------------------------------------------------------------------------*/
// here reg is including the write bit set, so reg == adr | WRITE
static uint8_t
write_single(uint8_t reg, uint8_t val)
{
  uint8_t s;
  /* sth (the button?) sets the MISO pin so lets reset all pins we need */
  SPI_PORT(SEL)  |= SPI_MISO | SPI_MOSI | SPI_SCL;
  SPI_PORT(SEL2) |= SPI_MISO | SPI_MOSI | SPI_SCL;

  CC2500_SPI_ENABLE();
  SPI_WRITE(reg);
  s = SPI_RXBUF;
  SPI_WRITE(val);
  CC2500_SPI_DISABLE();
  return s;
}
/*---------------------------------------------------------------------------*/
static void
init_chip(void)
{
  int i;
  // note, logic seems flaky, may be off, refer to the other logic captures
  uint8_t regs[] = {0x00, 0x02, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
                    0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                    0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21,
                    0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2C, 0x2D,
                    0x2E, 0x7E};
  uint8_t values[] = {0x2D, 0x06, 0xFF, 0x04, 0x05, 0x01, 0x10, 0x09, 0x00, 0x5D,
                      0x93, 0xB1, 0x2D, 0x3B, 0x73, 0xA2, 0xF8, 0x01, 0x07, 0x30,
                      0x18, 0x1D, 0x1C, 0xC7, 0x00, 0xB2, 0x87, 0x6B, 0xF8, 0xB6,
                      0x10, 0xEA, 0x0A, 0x00, 0x11, 0x41, 0x00, 0x59, 0x88, 0x31,
                      0x0B, 0xFF};
#define REGSET_LEN 42
  for(i = 0; i < REGSET_LEN; i++) {
    write_single(regs[i], values[i]);
  }
  write_single(PATABLE, 0xff);
}
/*---------------------------------------------------------------------------*/
static void
send_setting_once(uint8_t *setting, int len)
{
  // the IKEA remote does, for each packet:
  // strobe idle
  // strobe flush tx fifo
  // burstwrite 8 bytes to fifo (0x7f)
  // strobe tx
  // then just 1.7 ms wait (no SPI, perhaps GDOx?)

  strobe(CC2500_SIDLE);
  strobe(CC2500_SFTX);
  write_burst(setting, len);
  strobe(CC2500_STX);
  while((status() & CC2500_STATUSBYTE_STATUSBITS) != CC2500_STATE_TX);
  while((status() & CC2500_STATUSBYTE_STATUSBITS) == CC2500_STATE_TX);
}
/*---------------------------------------------------------------------------*/
#define NUM_TRANSMISSIONS     50

static void
send_setting(uint8_t setting)
{
  int i;
  uint8_t spibuf[9] = {
    0x7f,
    0x06,
    0x55,
    0x01,
    0x6a,
    0x99,
    0x01, // <- setting {0, 35, 100%} == {0x01, 0x02, 0x03}
    0xaa,
    0xff
  };
  /* change the 'setting'-byte */
  spibuf[6] = setting;

  // the IKEA remote does, for each press of the remote button:
  watchdog_periodic();
  strobe(CC2500_SRES);
  init_chip();
  for(i = 0; i < NUM_TRANSMISSIONS; i++) {
    watchdog_periodic();
    send_setting_once(spibuf, 9);

    while(/*SENDING*/0) {
    }
  }
  strobe(CC2500_SPWD);
}
/*---------------------------------------------------------------------------*/
PROCESS(ikea_process, "My Process");
AUTOSTART_PROCESSES(&ikea_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ikea_process, ev, data)
{
  PROCESS_BEGIN();
  static struct etimer et;
  static uint8_t setting = 0;

  while(1) {
    /* toggle settings every 2 seconds */
    setting++;
    if(setting > 3) {
      setting = 0;
    }
    send_setting(setting);

    etimer_set(&et, CLOCK_SECOND * 2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
