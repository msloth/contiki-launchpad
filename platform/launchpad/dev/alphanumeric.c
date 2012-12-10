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
 *         Launchpad driver for Sparkfun electronics Alphanumeric display
 * \author
 *        Marcus Lunden <marcus.lunden@gmail.com>
 *        based on:
 *        "AlphaNumeric_Driver.cpp - Arduino library to control a string of
 *         AlphaNumeric Display Drivers.
 *         License: Beerware."
 *         See <https://www.sparkfun.com/products/10103>
 *        
 */

#include "contiki.h"
#include "sparkfun-alphanumeric.h"
/*---------------------------------------------------------------------------*/
void
alphanum_init(void)
{
  ALPHANUM_PORT(IE) &= ~ALPHANUM_PINS_ALL;
  ALPHANUM_PORT(DIR) |= ALPHANUM_PINS_ALL;
  ALPHANUM_PORT(SEL) &= ~ALPHANUM_PINS_ALL;
  ALPHANUM_PORT(SEL2) &= ~ALPHANUM_PINS_ALL;
  ALPHANUM_PORT(OUT) &= ~ALPHANUM_PINS_ALL;
  ALPHANUM_PORT(OUT) |= ALPHANUM_LE_PIN;
  alphanum_clear();
}
/*---------------------------------------------------------------------------*/
/* turn on display */
void
alphanum_on(void)
{
  ALPHANUM_PORT(OUT) &= ~ALPHANUM_OE_PIN;
}
/*---------------------------------------------------------------------------*/
/* turn off displays */
void 
alphanum_off(void)
{
  ALPHANUM_PORT(OUT) |= ALPHANUM_OE_PIN;
}
/*---------------------------------------------------------------------------*/
void 
alphanum_scroll(char * string, int time)
{
  int i = 0;
  while(string[i] != '\0') {
    shift16(createShiftData(string[i]));
    delay(time);
    i++;
  }
}
/*---------------------------------------------------------------------------*/
void
alphanum_clear(void)
{
  for(int i = 0; i < ALPHANUM_DISPLAYS; i++) {
    shift16(0);
  }
}
/*---------------------------------------------------------------------------*/
void
alphanum_print(char printchar)
{
  uint16_t shiftData = alphanum_char_to_shift(toPrint);
  alphanum_shift16(shiftData);
}
/*---------------------------------------------------------------------------*/
void
alphanum_shift16(uint16_t data)
{
  shiftOut(_SDIpin, _CLKpin, MSBFIRST, (data>>8));
  shiftOut(_SDIpin, _CLKpin, MSBFIRST, (data&0x00FF));
}
/*---------------------------------------------------------------------------*/
uint16_t
alphanum_char_to_shift(char character)
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
