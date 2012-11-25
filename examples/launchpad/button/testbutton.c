
/* Testing button example
 * Author: Marcus Lunden <marcus.lunden@gmail.com>
 * expected result from running it:
 *   When pressing the button, the red LED will toggle until enough presses,
 *   then the green blinks stops and the button becomes unresponsive.
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/button.h"

/*
 * how to read a Contiki program: start by finding AUTOSTART_PROCESSES(...).
 * The first process in that list is started first. Then, if it starts a new
 * process by process_start() execution flow continues in that process until
 * any PROCESS_YIELD_* or PROCESS_WAIT_*.
 */
/* -------------------------------------------------------------------------- */
PROCESS(button_process, "Button process");
PROCESS(blink_process, "Blink process");
AUTOSTART_PROCESSES(&blink_process, &button_process);
/*---------------------------------------------------------------------------*/
/* This process handles button presses; up until 5 presses it toggles red LED
after that it kills the blink process if the button is pressed again. */
static struct etimer etr;

PROCESS_THREAD(button_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();
  static uint8_t btnpresses = 0;
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == button_event && (*((uint8_t*)data) & BUTTON_2));
    leds_toggle(LEDS_RED);
    btnpresses++;
    if(btnpresses == 5) {
      process_exit(&blink_process);
    }
  }
  PROCESS_END();
}
/*--------------------------------------------------------------------------*/
/* this process periodically blinks an LED */
static struct etimer et;

PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    leds_toggle(LEDS_GREEN);
    etimer_set(&et, CLOCK_SECOND/8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
