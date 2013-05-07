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
 *         Button drivers header file
 * \author
 *         Marcus Linderoth <linderoth.marcus@gmail.com>
 */


#ifndef BUTTON_H
#define BUTTON_H

/* P1.3 is the switch 2 on the Launchpad PCB, but other pins can be used. */
#define BUTTON_PORT(type)     P1##type

/* the switch on the LP */
#define BUTTON_2              (1<<3)

/* add all defined pins here, like this: (BUTTON_2 | BUTTON_HOME) */
#define BUTTON_PINS           (BUTTON_2)

/* convenience macro used for checking what button was pressed */
// XXX not working
//#define BUTTON_IS(x)          ((uint8_t)*data & x)

/*
  this example shows how you define more buttons; NB they all have to be on
  the same port (eg P1).
    #define TEST_SW               (1<<4)
    #define BUTTON_PINS           (SWITCH_2 | TEST_SW)
    ...
    PROCESS_WAIT_EVENT_UNTIL(ev == button_event && BUTTON_IS(SwITCH_2));
*/

process_event_t   button_event;

void              button_init(void);
uint8_t           button_pressed(void);

#endif /* BUTTON_H */
