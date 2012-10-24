/**
 * \addtogroup launchpad-examples
 *
 * @{
 */

/**
 * \file
 *    readadc-poll.c
 * \author
 *    Marcus Lunden <marcus.lunden@gmail.com>
 * \brief
 *    ADC reading example. One process periodically reads the ADC on one channel
 *    and updates a blink speed variable. Another sets a LED blink according to
 *    that variable.
 *    
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/adc.h"

static uint16_t adcval;
static struct etimer et;
/*--------------------------------------------------------------------------*/
PROCESS(adc_reading_process, "ADC reading process");
AUTOSTART_PROCESSES(&adc_reading_process);
/* -------------------------------------------------------------------------- */
static void
adc_pollhandler(void)
{
  printf("ADC: %u\n", adcval);
  leds_toggle(LEDS_GREEN);
  if(adcval > 500) {
    leds_on(LEDS_RED);
  } else {
    leds_off(LEDS_RED);
  }
}
/*--------------------------------------------------------------------------*/
PROCESS_THREAD(adc_reading_process, ev, data)
{
  PROCESS_POLLHANDLER(adc_pollhandler());
  PROCESS_BEGIN();

  leds_off(LEDS_ALL);

  while(1) {
    /* read analog in A7, store the result in adcval and poll this process when done */
    adc_get_poll(A7, &adcval, PROCESS_CURRENT());
    etimer_set(&et, CLOCK_SECOND/8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/** @} */
