
#ifndef __CC2500_H__
#define __CC2500_H__


#include "contiki.h"
#include "dev/spi.h"
#include "dev/radio.h"
#include "dev/cc2500-const.h"

/*--------------------------------------------------------------------------*/
#define CC2500_MAX_PACKET_LEN       63   // XXX 64?
/*
 * The transmission power setting refers to the vector of settings for txp; 
 * their corresponding dBm value is this (starting with 0x50==-30)
 *    {-30, -26, -18, -10, -4, +1}
 * hence the txp is used like this:
 *  packetbuf_attr(TRANSMISSION_POWER, CC2500_TXPOWER_MAX);
 *  packetbuf_attr(TRANSMISSION_POWER, 3);
 */
uint8_t cc2500_txp[] = {0x50, 0xC0, 0x93, 0x97, 0xA9, 0xFF};
#define CC2500_TXPOWER(x)           cc2500_txp[x]
#define CC2500_TXPOWER_MAX          5
#define CC2500_TXPOWER_MIN          0

/* SPI interface and helper functionality */
#define CC2500_SPI_PORT(type)       P1##type      
#define CC2500_SPI_ENABLE_PIN       (1<<4)
#define CC2500_CSN_PORT(type)       P1##type      
#define CC2500_SPI_ENABLE()         (CC2500_CSN_PORT(OUT) &= ~CC2500_SPI_ENABLE_PIN)
#define CC2500_SPI_DISABLE()        (CC2500_CSN_PORT(OUT) |=  CC2500_SPI_ENABLE_PIN)


/*
  this is the minimum required for the radio driver, must be implemented or stubbed
  cc2500_init,
  cc2500_prepare,
  cc2500_transmit,
  cc2500_send,
  cc2500_read,
  cc2500_cca,
  cc2500_receiving_packet,
  pending_packet,
  cc2500_on,
  cc2500_off,
*/

/*
  cc2420 functions that are not implemented for cc2500
  int cc2500_get_txpower(void);
  void cc2500_set_cca_threshold(int value);
  int cc2500_set_channel(int channel);
  int cc2500_get_channel(void);
  int cc2500_rssi(void);
  void cc2500_set_pan_addr(unsigned pan,
                                  unsigned addr,
                                  const uint8_t *ieee_addr);
*/
extern const struct radio_driver cc2500_driver;
/*--------------------------------------------------------------------------*/
int     cc2500_init(void);
void    cc2500_set_txpower(uint8_t power);
int     cc2500_interrupt(void);
int     cc2500_on(void);
int     cc2500_off(void);

uint8_t cc2500_strobe(uint8_t strobe);
uint8_t cc2500_read_single(uint8_t adr);
uint8_t cc2500_read_burst(uint8_t adr, uint8_t *dest, uint8_t len);
uint8_t cc2500_write_single(uint8_t adr, uint8_t data);
uint8_t cc2500_write_burst(uint8_t adr, uint8_t *src, uint8_t len);
/*--------------------------------------------------------------------------*/
#if 0
#define CC2500_STROBE(s)                                   \
  do {                                                  \
    CC2500_SPI_ENABLE();                                \
    SPI_WRITE(s);                                       \
    CC2500_SPI_DISABLE();                               \
  } while (0)

/* Write to a register in the CC2500                         */
/* Note: the SPI_WRITE(0) seems to be needed for getting the */
/* write reg working on the Z1 / MSP430X platform            */
#define CC2500_WRITE_REG(adr,data)                              \
  do {                                                       \
    CC2500_SPI_ENABLE();                                     \
    SPI_WRITE_FAST(adr);                                     \
    SPI_WRITE_FAST((uint8_t)((data) >> 8));                  \
    SPI_WRITE_FAST((uint8_t)(data & 0xff));                  \
    SPI_WAITFORTx_ENDED();                                   \
    SPI_WRITE(0);                                            \
    CC2500_SPI_DISABLE();                                    \
  } while(0)

/* Read a register in the CC2500 */
#define CC2500_READ_REG(adr,data)                          \
  do {                                                  \
    CC2500_SPI_ENABLE();                                \
    SPI_WRITE(adr | 0x40);                              \
    data = (uint8_t)SPI_RXBUF;                          \
    SPI_TXBUF = 0;                                      \
    SPI_WAITFOREORx();                                  \
    data = SPI_RXBUF << 8;                              \
    SPI_TXBUF = 0;                                      \
    SPI_WAITFOREORx();                                  \
    data |= SPI_RXBUF;                                  \
    CC2500_SPI_DISABLE();                               \
  } while(0)

#define CC2500_READ_FIFO_BYTE(data)                        \
  do {                                                  \
    CC2500_SPI_ENABLE();                                \
    SPI_WRITE(CC2500_RXFIFO | 0x40);                    \
    (void)SPI_RXBUF;                                    \
    SPI_READ(data);                                     \
    clock_delay(1);                                     \
    CC2500_SPI_DISABLE();                               \
  } while(0)

#define CC2500_READ_FIFO_BUF(buffer,count)                                 \
  do {                                                                  \
    uint8_t i;                                                          \
    CC2500_SPI_ENABLE();                                                \
    SPI_WRITE(CC2500_RXFIFO | 0x40);                                    \
    (void)SPI_RXBUF;                                                    \
    for(i = 0; i < (count); i++) {                                      \
      SPI_READ(((uint8_t *)(buffer))[i]);                               \
    }                                                                   \
    clock_delay(1);                                                     \
    CC2500_SPI_DISABLE();                                               \
  } while(0)

#define CC2500_WRITE_FIFO_BUF(buffer,count)                                \
  do {                                                                  \
    uint8_t i;                                                          \
    CC2500_SPI_ENABLE();                                                \
    SPI_WRITE_FAST(CC2500_TXFIFO);                                           \
    for(i = 0; i < (count); i++) {                                      \
      SPI_WRITE_FAST(((uint8_t *)(buffer))[i]);                              \
    }                                                                   \
    SPI_WAITFORTx_ENDED();                                              \
    CC2500_SPI_DISABLE();                                               \
  } while(0)
#endif

#endif /* __CC2500_H__ */

