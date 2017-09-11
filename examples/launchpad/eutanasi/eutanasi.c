#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
/*---------------------------------------------------------------------------*/
/*
 * Perform some stuff, then go to eternal sleep that can only be fixed by
 * POR or hardware reset.
 *
 * The purpose of this is to be in mostly either off (as in no power supply), or
 * in deep sleep. Basically,
 *   off -> on and doing something -> deep sleep (until off again)
 */
/* -------------------------------------------------------------------------- */
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
/* we want this to do something a while, then go to eternal sleep. */
PROCESS_THREAD(blink_process, ev, data)
{
  PROCESS_BEGIN();

  while(1) {
    static struct etimer et;
    static int onoff;

    /* before sleep, we blink a little */
    onoff = !onoff;
    if(onoff) {
      leds_on(LEDS_ALL);
    } else {
      leds_off(LEDS_ALL);
    }
    etimer_set(&et, CLOCK_SECOND / 8);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

#define TIME_BEFORE_ETERNAL_SLEEP (10) // seconds
    if(clock_seconds() >= TIME_BEFORE_ETERNAL_SLEEP) {
      /* turn off all LEDs to be as power efficient as possible */
      leds_off(LEDS_ALL);

      /* don't wake up from the watchdog or other interrupts (eg timers) */
      watchdog_stop();
      dint();

      /* enter low-power mode */
      LPM3;
      /* ... and rest in peace; only reset/POR can resurrect us now */
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
