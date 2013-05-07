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
 *         Clock-pwm drivers for MSP430G2xxx chips. This driver is a hack-on to
 *          give the msp430g2452 one PWM channel and the 2553 an additional.
 *          This is slightly more easy to use than the pwm module as the timer
 *          is configured by the clock. Simply use, for pwm on pin 1.6
 *            clock_pwm_confpin(6);               // sets all up for pin 1.6
 *            clock_pwm_on(25);                   // starts 25% duty cycle
 *            clock_pwm_on_pulsetime(100);        // 100 ticks duty cycle
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */

#ifndef __SIMPLE_PWM_H__
#define __SIMPLE_PWM_H__

#include <stdio.h>
#include "contiki.h"

#define SIMPLE_PWM_SECOND        32768UL

/* configure freq and port */
#ifdef SIMPLE_PWM_CONF_FREQ
#define SIMPLE_PWM_FREQ          SIMPLE_PWM_CONF_FREQ
#else
#define SIMPLE_PWM_FREQ          128
#endif

/* there are at least two approaches to this: use the timer hardware mapped pins,
or use arbitrary pins. Hw-mapped is leaner but has less flexibility. */
#ifdef SIMPLE_PWM_CONF_PORT
#define SIMPLE_PWM_PORT(type)          SIMPLE_PWM_CONF_PORT(type)
#else
#define SIMPLE_PWM_PORT(type)          P1##type
#endif

void      simple_pwm_confpin(uint8_t pin);
void      simple_pwm_pulsetime(uint16_t finetime);
void      simple_pwm_on(uint8_t dc);
void      simple_pwm_off(void);

#endif /* __SIMPLE_PWM_H__ */
