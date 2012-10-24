/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
#include "contiki.h"
#include "lib/sensors.h"
#include "dev/hwconf.h"
#include "dev/button-sensor.h"
#include "isr_compat.h"

const struct sensors_sensor button_sensor;

static struct timer debouncetimer;
static int status(int type);

#define BUTTON_PORT(type) P1##type
#define BUTTON_PIN        3
#define DEBOUNCE_TIME     (CLOCK_SECOND/8)

#define BUTTON_IRQ_ON()   (BUTTON_PORT(IE) & BV(BUTTON_PIN))

/*---------------------------------------------------------------------------*/
#if 0
ISR(PORT1, irq_p1)
{
  if(BUTTON_PORT(IFG) & BV(BUTTON_PIN)) {
    if(timer_expired(&debouncetimer)) {
      timer_set(&debouncetimer, DEBOUNCE_TIME);
      sensors_changed(&button_sensor);
      LPM4_EXIT;
    }
  }
  BUTTON_PORT(IFG) &=~ BV(BUTTON_PIN);
}
#endif
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  return BUTTON_PORT(IN) & BV(BUTTON_PIN) || !timer_expired(&debouncetimer);
}
/*---------------------------------------------------------------------------*/
  
static int
configure(int type, int c)
{
  switch (type) {
    case SENSORS_HW_INIT:
      /* called at start up; init button sensor */
      BUTTON_PORT(DIR) &= ~BV(BUTTON_PIN);
      BUTTON_PORT(OUT) |= BV(BUTTON_PIN);
      #if BOARD_OLD_REVISION
        /* the old revision of boards have a resistor R34 in place for the button; if
         * your board is such, then this becomes necessary.  */
      BUTTON_PORT(REN) |= BV(BUTTON_PIN);
      #endif
      BUTTON_PORT(IES) |= BV(BUTTON_PIN);
      BUTTON_PORT(IFG) &= ~BV(BUTTON_PIN);
      BUTTON_PORT(IE) |= BV(BUTTON_PIN);
      break;

    case SENSORS_ACTIVE:
      /* called at SENSORS_ACTIVATE (c=1) or SENSORS_DEACTIVATE (c=0) */
      if (c) {
        if (BUTTON_IRQ_ON()) {
	        timer_set(&debouncetimer, 0);
          BUTTON_PORT(IE) |= BV(BUTTON_PIN);
        }
      } else {
        BUTTON_PORT(IE) &= ~BV(BUTTON_PIN);
      }
      return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch (type) {
    case SENSORS_ACTIVE:
    case SENSORS_READY:
    return BUTTON_IRQ_ON();
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, BUTTON_SENSOR, value, configure, status);
