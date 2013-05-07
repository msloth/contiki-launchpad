/*
 * Copyright (c) 2013, Marcus Linderoth, http://forfunandprof.it
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * \file
 *         Servo motor with emergency button example
 *          Demonstrates using the PWM API with a different frequency set, and
 *          setting PWM with absolute time rather than duty cycle.
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/leds.h"
#include "dev/servo.h"
#include "dev/button.h"

/*
 * One process moves the servo arm back and forth through setting PWM to the
 * servo control pin accordingly. Another process acts as an emergency stop control
 * and stops the motor if the button is pressed. To restart, reset the Launchpad.
 *
 * The settings for the servo PWM are found by taking the PWM clock speed, which
 * is 32768 Hz, and the minimum and maximum control pulse lengths (ca 0.8..2.0 ms)
 *    SERVO_MAX = 2*(32768/1000) ~= 65
 *    SERVO_MIN = 0.8*(32768/1000) ~= 26
 *
 */

/* define port and pin for servo motor */
#define PWM_CONF_PORT         P1
#define SERVOCONTROL_PIN      (0)
/*---------------------------------------------------------------------------*/
PROCESS(button_process, "Button catcher");
PROCESS(servo_process, "Servo motor control process");
AUTOSTART_PROCESSES(&servo_process, &button_process);
/*---------------------------------------------------------------------------*/
/* this process controls the servo arm back and forth, toggling green LED at arm
    endpoints */
static struct etimer etr;

PROCESS_THREAD(servo_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  static uint8_t i = 1;
  static uint8_t up = 1;

  PROCESS_BEGIN();

  servo_init(SERVOCONTROL_PIN);
  while(1) {
    /* find new servo position control setting; go between 0..180 deg and back */
    if(up) {
      i++;
      if(i == 180) {
        up = 0;
        leds_toggle(LEDS_GREEN);
        /* wait a little while */
        servo_off();
        etimer_set(&etr, CLOCK_SECOND/2);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
      }
    } else {
      i--;
      if(i == 0) {
        up = 1;
        leds_toggle(LEDS_GREEN);
        /* wait a little while */
        servo_off();
        etimer_set(&etr, CLOCK_SECOND/2);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
      }
    }

    /* set new position */
    servo_set_position(i);

    /* wait a little while */
    etimer_set(&etr, CLOCK_SECOND/16);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* Kill the servo motor with the push of the button. */
PROCESS_THREAD(button_process, ev, data)
{
  PROCESS_POLLHANDLER();
  PROCESS_EXITHANDLER();
  PROCESS_BEGIN();

  /* when the button is pressed, kill the servo! */
  PROCESS_WAIT_EVENT_UNTIL(ev == button_event);
  servo_off();
  process_exit(&servo_process);

  /* then blink indefinitely */
  leds_off(LEDS_ALL);
  printf("[%u] Emergency stop pressed! Motor stopped.\n", clock_seconds());
  while(1) {
    leds_toggle(LEDS_ALL);
    etimer_set(&etr, CLOCK_SECOND/2);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&etr));
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
