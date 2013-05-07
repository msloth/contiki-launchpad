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
 *         Launchpad PWM drivers header file
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */


#ifndef __PWM_H__
#define __PWM_H__

/*--------------------------------------------------------------------------*/
/* PWM module is sourced from external crystal osc @ 32.768 kHz, hence do not change this */
#define PWM_SECOND        32768UL

/* configure freq and port */
#ifdef PWM_CONF_FREQ
#define PWM_FREQ          PWM_CONF_FREQ
#else
#define PWM_FREQ          128
#endif

/* there are at least two approaches to this: use the timer hardware mapped pins,
or use arbitrary pins. Hw-mapped is leaner but has less flexibility. */
#ifdef PWM_CONF_PORT
#define PWM_PORT(type)          PWM_CONF_PORT(type)
#else
#define PWM_PORT(type)          P1##type
#endif
/*--------------------------------------------------------------------------*/
void      pwm_init(uint16_t freq);
uint8_t   pwm_dc(uint8_t pwmdevice);
uint16_t  pwm_period(void);

void      pwm_confpin(uint8_t pin);
void      pwm_on(uint8_t pwmdevice, uint8_t pin, uint8_t dc);
void      pwm_pulsetime(uint8_t pwmdevice, uint8_t pin, uint16_t finetime);
void      pwm_off(uint8_t pwmdevice);
void      pwm_all_off(void);
/*--------------------------------------------------------------------------*/
#endif /* __PWM_H__ */


