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
 *         Launchpad driver for Sparkfun electronics Alphanumeric display
 * \author
 *        Marcus Linderoth <linderoth.marcus@gmail.com>
 *        based on:
 *        "AlphaNumeric_Driver.cpp - Arduino library to control a string of
 *         AlphaNumeric Display Drivers.
 *         License: Beerware."
 *         See https://www.sparkfun.com/products/10103
 *
 */

#include <stdio.h>
#include "contiki.h"
#include "alphanumeric.h"
/*---------------------------------------------------------------------------*/
static void       shift_out(uint16_t data);
static uint16_t   char_to_shift(char character);
/*---------------------------------------------------------------------------*/
/* init ports and pins, clear displays */

/*
    Applications where the latches are bypassed (LE tied high)
    require that the OE(Output Enable) input be high during serial
    data entry. When OE is high, the output sink drivers are disabled
    (off). The data stored in the latches is not affected by the state
    of OE. With OE active (low), the outputs are controlled by the
    state of their respective latches.

    High data input rate: 30 MHz

    3.0 to 5.5 V logic supply range

    Rext = 4.7k now
    The maximum channel output current can be calculated as:
        IO(max) = (18483.1/ REXT) + 0.67 , for VDD = 3.0 to 3.6 V ,
        == 4.6 mA @ 4.7k
    or
        IO(max) = (18841.2/ REXT) + 0.68 , for VDD = 4.5 to 5.5 V ,
        == 4.7 mA @ 4.7k
    where REXT is the value of the user-selected external resistor,
    which should not be less than 374 Ω.
    Adding another resistor of 4.7k makes an IOmax at 3.0--3.6 == 8.54 mA per segment
*/
void
alphanum_init(void)
{
  ALPHANUM_PORT(IE) &= ~ALPHANUM_PINS_ALL;
  ALPHANUM_PORT(DIR) |= ALPHANUM_PINS_ALL;
  ALPHANUM_PORT(SEL) &= ~ALPHANUM_PINS_ALL;
  ALPHANUM_PORT(SEL2) &= ~ALPHANUM_PINS_ALL;
  ALPHANUM_PORT(OUT) &= ~ALPHANUM_PINS_ALL;
  alphanum_clear();
  alphanum_on();
}
/*---------------------------------------------------------------------------*/
/* turn on all displays */
void
alphanum_on(void)
{
  /* Output enable is active low */
  ALPHANUM_PORT(OUT) &= ~ALPHANUM_OE_PIN;
}
/*---------------------------------------------------------------------------*/
/* turn off all displays */
void
alphanum_off(void)
{
  /* Output enable is active low */
  ALPHANUM_PORT(OUT) |= ALPHANUM_OE_PIN;
}
/*---------------------------------------------------------------------------*/
void
alphanum_shift(uint16_t d)
{
  shift_out(d);
}
/*--------------------------------------------------------------------------*/











/* scroll a text over the displays */
// XXX to be implemented as a process to ensure we play nice and not hog CPU while delay

/*void */
/*alphanum_scroll(char *string, int time)*/
/*{*/
/*  int i = 0;*/
/*  while(string[i] != '\0') {*/
/*    shift_out(createShiftData(string[i]));*/
/*    delay(time);*/
/*    i++;*/
/*  }*/
/*}*/
/*---------------------------------------------------------------------------*/
/* clear all displays */
void
alphanum_clear(void)
{
  uint8_t i;
  for(i = 0; i < ALPHANUM_DISPLAYS; i++) {
    shift_out(0);
  }
}
/*---------------------------------------------------------------------------*/
/* prints out one character on the displays */
void
alphanum_print_char(char ch)
{
  shift_out(char_to_shift(ch));
}
/*---------------------------------------------------------------------------*/
/* prints out a string on the displays, no scrolling. */
// XXX is now backwards! TODO
void
alphanum_print_string(char *str)
{
  uint8_t i = 0;
  while(str[i] != '\0' && i < ALPHANUM_DISPLAYS) {
    shift_out(char_to_shift(str[i]));
    i++;
  }
}
/*---------------------------------------------------------------------------*/
/* shift out one character (eg a..z, A..Z, 1..0) to the display */
static void
shift_out(uint16_t data)
{
  uint8_t i;
/*  uint8_t oesave = 0;*/
  uint16_t s = 0x8000;    /* MSB first */

  ALPHANUM_PORT(OUT) &= ~ALPHANUM_CLK_PIN;
#if ALPHANUM_LE_PIN != 0
  ALPHANUM_PORT(OUT) &=~ ALPHANUM_LE_PIN;
#endif
  /* OE must be high during transfers if LE tied high; this temporarily disables
    output. */
    //XXX no, wasn't necessary.
/*  oesave = ALPHANUM_PORT(OUT) & ALPHANUM_OE_PIN;*/
/*  ALPHANUM_PORT(OUT) |= ALPHANUM_OE_PIN;*/

  for(i = 0; i < 16; i += 1) {
    if(data & s) {
      ALPHANUM_PORT(OUT) |= ALPHANUM_SD_PIN;
    } else {
      ALPHANUM_PORT(OUT) &= ~ALPHANUM_SD_PIN;
    }
    ALPHANUM_PORT(OUT) |= ALPHANUM_CLK_PIN;
    s = s >> 1;
    ALPHANUM_PORT(OUT) &= ~ALPHANUM_CLK_PIN;
  }
#if ALPHANUM_LE_PIN != 0
  ALPHANUM_PORT(OUT) |= ALPHANUM_LE_PIN;
#endif

  /* re-set the OE pin to what it was before */
/*  if(!oesave) {*/
/*    ALPHANUM_PORT(OUT) &= ~ALPHANUM_OE_PIN;*/
/*  }*/
}
/*---------------------------------------------------------------------------*/
/* convert an 8-bit character to the 16-bit code needed by the display driver IC */
static uint16_t
char_to_shift(char character)
{
  switch(character) {
    case '0':
    case 0:
      return ZERO;
      break;

    case '1':
    case 1:
      return ONE;
      break;

    case '2':
    case 2:
      return TWO;
      break;

    case '3':
    case 3:
      return THREE;
      break;

    case '4':
    case 4:
      return FOUR;
      break;

    case '5':
    case 5:
      return FIVE;
      break;

    case '6':
    case 6:
      return SIX;
      break;

    case '7':
    case 7:
      return SEVEN;
      break;

    case '8':
    case 8:
      return EIGHT;
      break;

    case '9':
    case 9:
      return NINE;
      break;

    case 'A':
    case 'a':
    case 10:
      return A_CHAR;
      break;

    case 'B':
    case 'b':
    case 11:
      return B_CHAR;
      break;

    case 'C':
    case 'c':
    case 12:
      return C_CHAR;
      break;

    case 'D':
    case 'd':
    case 13:
      return D_CHAR;
      break;

    case 'E':
    case 'e':
    case 14:
      return E_CHAR;
      break;

    case 'F':
    case 'f':
    case 15:
      return F_CHAR;
      break;

    case 'G':
    case 'g':
      return G_CHAR;
      break;

    case 'H':
    case 'h':
      return H_CHAR;
      break;

    case 'I':
    case 'i':
      return I_CHAR;
      break;

    case 'J':
    case 'j':
      return J_CHAR;
      break;

    case 'K':
    case 'k':
      return K_CHAR;
      break;

    case 'L':
    case 'l':
      return L_CHAR;
      break;

    case 'M':
    case 'm':
      return M_CHAR;
      break;

    case 'N':
    case 'n':
      return N_CHAR;
      break;

    case 'O':
    case 'o':
      return O_CHAR;
      break;

    case 'P':
    case 'p':
      return P_CHAR;
      break;

    case 'Q':
    case 'q':
      return Q_CHAR;
      break;

    case 'R':
    case 'r':
      return R_CHAR;
      break;

    case 'S':
    case 's':
      return S_CHAR;
      break;

    case 'T':
    case 't':
      return T_CHAR;
      break;

    case 'U':
    case 'u':
      return U_CHAR;
      break;

    case 'V':
    case 'v':
      return V_CHAR;
      break;

    case 'W':
    case 'w':
      return W_CHAR;
      break;

    case 'X':
    case 'x':
      return X_CHAR;
      break;

    case 'Y':
    case 'y':
      return Y_CHAR;
      break;

    case 'Z':
    case 'z':
      return Z_CHAR;
      break;

    case ' ':
      return 0;
      break;

    case '!':
      return EXCLAMATION;
      break;

    case '#':
      return (uint16_t) 0xFFFF;
      break;

    case '$':
      return DOLLAR;
      break;

    case '%':
      return PERCENT;
      break;

    case '^':
      return CARROT;
      break;

    case '&':
      return AMPERSAND;
      break;

    case '*':
      return ASTERISK;
      break;

    case '(':
      return LPAREN;
      break;

    case ')':
      return RPAREN;
      break;

    case '-':
      return MINUS;
      break;

    case '_':
      return UNDERSCORE;
      break;

    case '+':
      return PLUSYSIGN;
      break;

    case '=':
      return EQUALS;
      break;

    case '>':
      return RARROW;
      break;

    case '<':
      return LARROW;
      break;

    case ',':
      return COMMA;
      break;

    case '/':
      return FSLASH;
      break;

    case '\\':
      return BSLASH;
      break;

    case '\'':
      return SINGLEQUOTE;
      break;

    case '"':
      return DOUBLEQUOTE;
      break;

    case 0X5B:
      return LBRACKET;
      break;

    case 0X5D:
      return RIBRACKET;
      break;

    case 0X7D:
      return LECURLY;
      break;

    case 0X7B:
      return RICURLY;
      break;

    case '|':
      return PIPE;
      break;

    case '~':
      return TILDE;
      break;

    case '`':
      return APOSTROPHE;
      break;

    case '@':
      return ATSIGN;
      break;

    case '?':
      return QUESTIONMARK;
      break;

    case ':':
      return COLON;
      break;

    case ';':
      return SEMICOLON;
      break;

    case '.':
      return PERIOD;
      break;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
