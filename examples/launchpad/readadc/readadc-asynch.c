/**
 * \addtogroup launchpad-examples
 *
 * @{
 */

/**
 * \file
 *    readadc-asynch.c
 * \author
 *    Marcus Lunden <marcus.lunden@gmail.com>
 * \brief
 *    ADC reading example. One process periodically reads the ADC on one channel
 *    and updates a blink speed variable, plus blinks an LED. Another sets a LED
 *    blink according to that variable. Program execution is not blocked while 
 *    ADC is converting.
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/adc.h"

static volatile uint16_t adc = 0;
/*--------------------------------------------------------------------------*/
PROCESS(adc_reading_process, "ADC reading process");
PROCESS(blink_process, "My Process");
AUTOSTART_PROCESSES(&adc_reading_process, &blink_process);
/*--------------------------------------------------------------------------*/
static uint16_t time = 0;
static struct etimer etb;
PROCESS_THREAD(blink_process, ev, data) {
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  while(1) {
    leds_toggle(LEDS_RED);
    if(adc > 0) {
      time = (CLOCK_SECOND*adc)/1024;
    } else {
      time = CLOCK_SECOND/64;
    }
    if(time == 0) {
      time = 1;
    }
    etimer_set(&etb, time);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etb));
  }
  PROCESS_END();
}
/*--------------------------------------------------------------------------*/
static struct etimer et;
PROCESS_THREAD(adc_reading_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    /*
     * read analog in A7, store the result in variable adc when done (no
     * notification will be given when so happens)
     */
    adc_get_noblock(A7, &adc);
    leds_toggle(LEDS_GREEN);
    etimer_set(&et, CLOCK_SECOND/8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    printf("ADC: %u\n", adc);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */
