
/* Blink example application,
 * Author: Marcus Lunden <marcus.lunden@gmail.com>
 * Demonstrates starting another process from one, setting timers, using LEDs.
 * expected result from running it:
 *   red LED flashing fast (toggling every 125 ms), green every second.
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"

/* -------------------------------------------------------------------------- */
PROCESS(red_process, "Red light process");
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
/* This process is started from the other process and blinks an LED fast.
*/
static struct etimer etr;
PROCESS_THREAD(red_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  while(1) {
    leds_toggle(LEDS_RED);
    etimer_set(&etr, CLOCK_SECOND/8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
  }
  PROCESS_END();
}

/*--------------------------------------------------------------------------*/
/* this process starts another process and then periodically blinks an LED */
static struct etimer et;

PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();

  etimer_set(&et, CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  process_start(&red_process, NULL);

  while(1) {
    /* unnecessary, messy but wanted sth else than what the red process 
     * is doing so instead of just toggling, turn on when even seconds and
     * turn off on uneven seconds.
     */
    if(clock_seconds() & 1) {
      leds_on(LEDS_GREEN);
    } else {
      leds_off(LEDS_GREEN);
    }
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

