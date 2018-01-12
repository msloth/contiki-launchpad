#include <stdlib.h>
#include <stdio.h>
#include "contiki.h"
#include "watchdog.h"
#include "leds.h"

#include <msp430.h>
#include <legacymsp430.h>
#include "i2c-slave.h"
/*---------------------------------------------------------------------------*/
PROCESS(startup_process, "My Process");
AUTOSTART_PROCESSES(&startup_process);
/*---------------------------------------------------------------------------*/
/*
 * This firmware and drivers act as an I2C slave device. It has a small command
 * set which allows both read and write access, to mimic a simple sensor.
 * It can be used to implement simple slave devices, or prototyping when the
 * intended slave device is not available.
 * 
 * This is based on slaa383.
 * NOTE: 10k external pull-ups are needed on SDA/SCL.
 * Also note that the callbacks are in interrupt context since the i2c driver
 * calls them directly from the i2c ISR. As such, minimal work should be done
 * in them.
 */
/*---------------------------------------------------------------------------*/
/*
 * we could/should instead use a separate variable to keep track of state,
 * however since it's just for this simple purpose, we just claim the command
 * code 0xff for this - meaning that no command can be 0xff. 
 */
#define WAITING_FOR_COMMAND 0xff

/* keep track of what the last command was, for the state machine */
static volatile uint8_t last_cmd = WAITING_FOR_COMMAND;

/* input and output buffers */
static volatile unsigned int i2c_in_buffer_ix;
static volatile uint8_t i2c_in_buffer[10];
static volatile unsigned int i2c_out_len; /* how many bytes the response will be */
static volatile uint8_t i2c_out_buffer[10];
/*---------------------------------------------------------------------------*/
/* these are the commands we respond to, except 0x88 they are copies from the tsl2561 sensor */
#define WRITE_TO_CONTROL_REGISTER_0     0x80 /* params necessary, one single byte */
#define WRITE_TO_CONTROL_REGISTER_1     0x81 /* params necessary, one single byte */
#define READ_FROM_LONG_REGS             0x88 /* params necessary, multi bytes */
#define READ_FROM_CH0                   0xAC /* no params necessary */
#define READ_FROM_CH1                   0xAE /* no params necessary */

#define READ_FROM_LONG_REGS_PARAM_LEN   3 /* we expect these many bytes as parameters */
/*---------------------------------------------------------------------------*/
/* callback for start condition */
static void
start_cb(void)
{
  /* we get a start condition == reset of the slave state machine */
  last_cmd = WAITING_FOR_COMMAND;
  i2c_in_buffer_ix = 0;
}
/*---------------------------------------------------------------------------*/
/*
 * callback for when we receive bytes in "write" mode, ie the r/w bit in the
 * first byte after the START condition (after the slave address) is 0.
 * 
 * these typically contain eg a command with parameters, or the address of a
 * register we want to read from in the next transaction.
 *
 * It can look like this,
 *   [] are from the master and triggers the receive callback (except the slave add+r/w)
 *   <> are from the slave and what the transmit callback puts in the tx output buffer
 *
 * reading from a register is two transactions:
 *   [slave address + w bit] - [register address we will read from]
 *   [slave address + r bit] - <data from slave 1> - <data from slave 2>
 *
 * or writing to a register, one transaction
 *   [slave address + w bit] - [register address we will write to] - [data 1] - [data 2] - [data 3]
 *
 */
static void
receive_cb(unsigned char b)
{
  /* this is what we receive in write mode, ie the master writes to us */
  if(last_cmd == WAITING_FOR_COMMAND) {
    /* THIS IS THE FIRST BYTE WE RECEIVE, the command byte */
    last_cmd = b;

    /* here, we can handle all the commands that need no parameters or additional data */
    if(last_cmd == READ_FROM_CH0 || last_cmd == READ_FROM_CH1) {
      /* these commands need no arguments, so we can prepare outdata now */
      i2c_out_buffer[0] = 0xab;
      i2c_out_buffer[1] = 0xde;
      i2c_out_len = 2;
     }

  } else if(last_cmd == READ_FROM_LONG_REGS) {
    /* this command expects a few more bytes of parameters, we store them here */
    i2c_in_buffer[i2c_in_buffer_ix] = b;
    i2c_in_buffer_ix++;
    if(i2c_in_buffer_ix == READ_FROM_LONG_REGS_PARAM_LEN) {
      /* we know all we need, handle the command by preparing the output buffer */
      /* here, we send back both a counter, and echo back what we got */
      static volatile uint8_t counter; /* just a funny counter to have some data to send */
      i2c_in_buffer_ix = 0;
      i2c_out_buffer[0] = counter++;
      i2c_out_buffer[1] = counter++;
      i2c_out_buffer[2] = counter++;
      i2c_out_buffer[3] = i2c_in_buffer[0];
      i2c_out_buffer[4] = i2c_in_buffer[1];
      i2c_out_buffer[5] = i2c_in_buffer[5];
      i2c_out_len = 6;
    }

  /* here handle setting the control registers, this should do sanity checking etc and not do it from this callback since they are called from an interrupt context */
  } else if(last_cmd == WRITE_TO_CONTROL_REGISTER_0) {
    // control_reg_setting_0 = b;
  } else if(last_cmd == WRITE_TO_CONTROL_REGISTER_1) {
    // control_reg_setting_1 = b;
  }
}
/*---------------------------------------------------------------------------*/
/* callback to transmit bytes */
static void
transmit_cb(unsigned char volatile *wbuffer)
{
  /* if caller reads more than intended, we return zeroes */
  if(i2c_out_len > 0) {
    *wbuffer = i2c_out_buffer[i2c_out_len - 1];
    i2c_out_len--;
  } else {
    *wbuffer = 0;
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(startup_process, ev, data)
{
  PROCESS_BEGIN();

  // WDTCTL = WDTPW + WDTHOLD;                      // Stop WDT
  // P1DIR |= BIT0;                                 // Set P1.0 to output direction
  // P1DIR &= ~BIT3;                                // Set P1.3 to input  direction
  // P1OUT &= ~BIT0;

#define SLAVE_ADDRESS       0x39
#define SDA_PIN             BIT7 // P1.7
#define SCL_PIN             BIT6 // P1.6
  i2c_slave_init(start_cb, transmit_cb, receive_cb, SLAVE_ADDRESS, SDA_PIN, SCL_PIN);

  // BCSCTL1 = CALBC1_16MHZ; // run at 16MHz, global interrupts enabled
  // DCOCTL  = CALDCO_16MHZ;
  __bis_SR_register(GIE);

  /* code is interrupt-driven, so we just nop away here */
  // while(1) __asm__("nop");

  static struct etimer et;
  while(1) {
    static int k;
    // if(k) {
    //   leds_on(LEDS_RED);
    // } else {
    //   leds_off(LEDS_RED);
    // }
    k = !k;
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }


  /* nothing more to do, the fw is interrupt-driven */
  PROCESS_WAIT_EVENT_UNTIL(0);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
