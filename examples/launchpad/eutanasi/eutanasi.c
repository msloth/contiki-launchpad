#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
/*---------------------------------------------------------------------------*/
/*
 * Perform some stuff, then go to eternal sleep that can only be disturbed by
 * POR or hardware reset.
 */
/* -------------------------------------------------------------------------- */
PROCESS(red_process, "Red light process");
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(red_process, ev, data)
{
  PROCESS_BEGIN();
  while(1) {
    static struct etimer etr;
    leds_toggle(LEDS_RED);

    etimer_set(&etr, CLOCK_SECOND/8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
  }
  PROCESS_END();
}
/*--------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {
    static struct etimer et;
    if(clock_seconds() & 1) {
      leds_on(LEDS_GREEN);
    } else {
      leds_off(LEDS_GREEN);
    }
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* some time after booting */
    if(clock_seconds() == 5) {
      /* start blinking red for 5 seconds before sleeping */
      process_start(&red_process, NULL);
      etimer_set(&et, CLOCK_SECOND * 5);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      /* turn off all LEDs to be as power efficient as possible */
      leds_off(LEDS_ALL);

      /* don't wake up from the watchdog or other interrupts (eg timers) */
      watchdog_stop();
      dint();

      /* rest in peace */
      LPM3;
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
