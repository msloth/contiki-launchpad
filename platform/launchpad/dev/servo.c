/*
 * Copyright (c) 2012
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
 */

/**
 * \file
 *         Simple servo driver
 *         The servo uses PWM-device 0 and sets the freq of the PWM to 64 Hz.
 * \author
 *         Marcus Lunden <marcus.lunden@gmail.com>
 */

#include <stdio.h>
#include "contiki.h"
#include "dev/servo.h"
#include "dev/pwm.h"
#include "dev/leds.h"

/* what pin is the servo control at? */
static uint8_t cpin;

/* the control PWM freq, between 50-100 Hz but use power of two so timer input
divider works; don't change this one */
#define SERVO_PWMFREQ         50


#define SERVO_MAX_DEG         180
/* PWM settings */
#define SERVO_MIN             26    /* 0 deg */
#define SERVO_MAX             65    /* 180 deg */
/*---------------------------------------------------------------------------*/
void
servo_init(uint8_t controlpin)
{
#if _MCU_ == 2553
  if(controlpin > 7) {
    return;
  }
  cpin = controlpin;
  P1DIR |= (1 << cpin);
  P1SEL &= ~((1 << cpin));
  P1SEL2 &= ~((1 << cpin));

  pwm_init(SERVO_PWMFREQ);
#endif    /* _MCU_ == 2553 */
}
/*---------------------------------------------------------------------------*/
void
servo_set_position(uint16_t position)
{
#if _MCU_ == 2553
  uint8_t ptime;
  if(position > SERVO_MAX_DEG) {
    /* error; halt motor and return */
    pwm_off(0);
    return;
  }

  /* convert position to absolute pulse time and set PWM */
  ptime = SERVO_MIN + ((SERVO_MAX - SERVO_MIN) * position)/SERVO_MAX_DEG;
  printf("Servo %u %u\n", position, ptime);
  if(ptime == 0) {
    /* handle rounding errors - ensure not zero PWM or the servo won't move */
    ptime = 1;
  }
  pwm_pulsetime(0, cpin, ptime);
#endif    /* _MCU_ == 2553 */
}
/*---------------------------------------------------------------------------*/
void
servo_off(void)
{
#if _MCU_ == 2553
  pwm_off(0);
#endif    /* _MCU_ == 2553 */
}
/*---------------------------------------------------------------------------*/

