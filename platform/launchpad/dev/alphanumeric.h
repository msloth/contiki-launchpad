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
 */

#ifndef __ALPHANUM_H__
#define __ALPHANUM_H__

void alphanum_init(void);
void alphanum_on(void);
void alphanum_off(void);
void alphanum_clear(void);
void alphanum_print_char(char ch);
void alphanum_print_string(char *str);
void alphanum_shift(uint16_t d);
//void alphanum_scroll(char *string, int time); // XXX coming later

/* define ports and pins for the latched serial LED driver on the display */
#define ALPHANUM_PORT(type)       P2##type
#define ALPHANUM_SD_PIN           (1<<3)
#define ALPHANUM_CLK_PIN          (1<<4)
#define ALPHANUM_OE_PIN           (1<<2)
/* the LE pin can be permanently tied high to save a GPIO, but then the shifting
  out might be visible. Set to zero if you permanently tied it to Vcc. */
#define ALPHANUM_LE_PIN           (1<<5)
//#define ALPHANUM_LE_PIN           (0)
#define ALPHANUM_PINS_ALL         (ALPHANUM_SD_PIN | ALPHANUM_CLK_PIN | ALPHANUM_LE_PIN | ALPHANUM_OE_PIN)

/* number of displays */
#define ALPHANUM_DISPLAYS         2

/* flipped by default */
#define FLIPDISPLAY               1

/* Bit position relating display pin to driver output pin */
#ifndef FLIPDISPLAY

#define D2_SEG    7 // 2 5
#define D1_SEG    10 // 22 18
#define E_SEG     11 // 21 17
#define F_SEG     13 // 16 15
#define A2_SEG    2 // 10 10
#define A1_SEG    15 // 14 13
#define B_SEG     3 // 9 9
#define C_SEG     5 // 4 7
#define G2_SEG    4 // 7 8
#define G1_SEG    12 // 19 16
#define L_SEG     6 // 3 6
#define M_SEG     8 // 24 20
#define N_SEG     9 // 23 19
#define H_SEG     14 // 15 14
#define J_SEG     0 // 12 12
#define K_SEG     1 // 11 11

#else   /* FLIPDISPLAY */

#define A1_SEG    7 // 2 5
#define A2_SEG    10 // 22 18
#define B_SEG     11 // 21 17
#define C_SEG     13 // 16 15
#define D1_SEG    2 // 10 10
#define D2_SEG    15 // 14 13
#define E_SEG     3 // 9 9
#define F_SEG     5 // 4 7
#define G1_SEG    4 // 7 8
#define G2_SEG    12 // 19 16
#define H_SEG     6 // 3 6
#define J_SEG     8 // 24 20
#define K_SEG     9 // 23 19
#define L_SEG     14 // 15 14
#define M_SEG     0 // 12 12
#define N_SEG     1 // 11 11

#endif    /* FLIPDISPLAY */

#define A_CHAR    (1 << G1_SEG) + (1 << G2_SEG) + (1 << E_SEG) + (1 << F_SEG) + (1 << A1_SEG) + (1 << A2_SEG) + (1 << B_SEG) + (1 << C_SEG)
#define B_CHAR    (1 << A1_SEG) + (1 << A2_SEG) + (1 << B_SEG) + (1 << C_SEG) + (1 << D2_SEG) + (1 << D1_SEG) + (1 << M_SEG) + (1 << J_SEG) + (1 << G1_SEG) + (1 << G2_SEG)
#define C_CHAR    (1 << A1_SEG) + (1 << A2_SEG) + (1 << D2_SEG) + (1 << D1_SEG) + (1 << E_SEG) + (1 << F_SEG)
#define D_CHAR    (1 << A1_SEG) + (1 << A2_SEG) + (1 << B_SEG) + (1 << C_SEG) + (1 << D2_SEG) + (1 << D1_SEG) + (1 << M_SEG) + (1 << J_SEG)
#define E_CHAR    (1 << A1_SEG) + (1 << A2_SEG) + (1 << D2_SEG) + (1 << D1_SEG) + (1 << E_SEG) + (1 << F_SEG) + (1 << G1_SEG) + (1 << G2_SEG)
#define F_CHAR    (1 << A1_SEG) + (1 << A2_SEG) + (1 << F_SEG) + (1 << G1_SEG) + (1 << E_SEG)
#define G_CHAR    (1 << A1_SEG) + (1 << A2_SEG) + (1 << F_SEG) + (1 << E_SEG) + (1 << D1_SEG) + (1 << D2_SEG) + (1 << C_SEG) + (1 << G2_SEG)
#define H_CHAR    (1 << F_SEG) + (1 << E_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << B_SEG) + (1 << C_SEG)
#define I_CHAR    (1 << A1_SEG) + (1 << A2_SEG) + (1 << J_SEG) + (1 << M_SEG) + (1 << D1_SEG) + (1 << D2_SEG)
#define J_CHAR    (1 << B_SEG) + (1 << C_SEG) + (1 << D1_SEG) + (1 << D2_SEG)
#define K_CHAR    (1 << F_SEG) + (1 << E_SEG) + (1 << G1_SEG) + (1 << K_SEG) + (1 << L_SEG)
#define L_CHAR    (1 << F_SEG) + (1 << E_SEG) + (1 << D1_SEG) + (1 << D2_SEG)
#define M_CHAR    (1 << E_SEG) + (1 << F_SEG) + (1 << H_SEG) + (1 << K_SEG) + (1 << B_SEG) + (1 << C_SEG)
#define N_CHAR    (1 << E_SEG) + (1 << F_SEG) + (1 << H_SEG) + (1 << L_SEG) + (1 << C_SEG) + (1 << B_SEG)
#define O_CHAR    (1 << F_SEG) + (1 << E_SEG) + (1 << D1_SEG) + (1 << D2_SEG) + (1 << C_SEG) + (1 << B_SEG) + (1 << A2_SEG) + (1 << A1_SEG)
#define P_CHAR    (1 << E_SEG) + (1 << F_SEG) + (1 << A1_SEG) + (1 << A2_SEG) + (1 << B_SEG) + (1 << G2_SEG) + (1 << G1_SEG)
#define Q_CHAR    (1 << F_SEG) + (1 << E_SEG) + (1 << D1_SEG) + (1 << D2_SEG) + (1 << C_SEG) + (1 << B_SEG) + (1 << A2_SEG) + (1 << A1_SEG) + (1 << L_SEG)
#define R_CHAR    (1 << E_SEG) + (1 << F_SEG) + (1 << A1_SEG) + (1 << A2_SEG) + (1 << B_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << L_SEG)
#define S_CHAR    (1 << A2_SEG) + (1 << A1_SEG) + (1 << F_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << C_SEG) + (1 << D2_SEG) + (1 << D1_SEG)
#define T_CHAR    (1 << A1_SEG) + (1 << A2_SEG) + (1 << J_SEG) + (1 << M_SEG)
#define U_CHAR    (1 << F_SEG) + (1 << E_SEG) + (1 << D1_SEG) + (1 << D2_SEG) + (1 << C_SEG) + (1 << B_SEG)
#define V_CHAR    (1 << F_SEG) + (1 << E_SEG) + (1 << N_SEG) + (1 << K_SEG)
#define W_CHAR    (1 << F_SEG) + (1 << E_SEG) + (1 << N_SEG) + (1 << L_SEG) + (1 << C_SEG) + (1 << B_SEG)
#define X_CHAR    (1 << H_SEG) + (1 << N_SEG) + (1 << L_SEG) + (1 << K_SEG)
#define Y_CHAR    (1 << H_SEG) + (1 << K_SEG) + (1 << M_SEG)
#define Z_CHAR    (1 << A1_SEG) + (1 << A2_SEG) + (1 << K_SEG) + (1 << N_SEG) + (1 << D1_SEG) + (1 << D2_SEG)

#define ONE       (1 << B_SEG) + (1 << C_SEG)
#define TWO       (1 << A1_SEG) + (1 << A2_SEG) + (1 << B_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << E_SEG) + (1 << D1_SEG) + (1 << D2_SEG)
#define THREE     (1 << A1_SEG) + (1 << A2_SEG) + (1 << B_SEG) + (1 << G2_SEG) + (1 << C_SEG) + (1 << D2_SEG) + (1 << D1_SEG)
#define FOUR      (1 << F_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << B_SEG) + (1 << C_SEG)
#define FIVE      (1 << A1_SEG) + (1 << A2_SEG) + (1 << F_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << C_SEG) + (1 << D2_SEG) + (1 << D1_SEG)
#define SIX       (1 << A2_SEG) + (1 << A1_SEG) + (1 << F_SEG) + (1 << G2_SEG) + (1 << G1_SEG) + (1 << C_SEG) + (1 << D2_SEG) + (1 << D1_SEG) + (1 << E_SEG)
#define SEVEN     (1 << A1_SEG) + (1 << A2_SEG) + (1 << B_SEG) + (1 << C_SEG)
#define EIGHT     (1 << A1_SEG) + (1 << A2_SEG) + (1 << F_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << B_SEG) + (1 << C_SEG) + (1 << D2_SEG) + (1 << D1_SEG) + (1 << E_SEG)
#define NINE      (1 << A1_SEG) + (1 << A2_SEG) + (1 << F_SEG) + (1 << B_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << C_SEG)
#define ZERO      (1 << F_SEG) + (1 << E_SEG) + (1 << D1_SEG) + (1 << D2_SEG) + (1 << C_SEG) + (1 << B_SEG) + (1 << A2_SEG) + (1 << A1_SEG)

#define EXCLAMATION   (1 << B_SEG) + (1 << C_SEG)
#define ATSIGN    (1 << D2_SEG) + (1 << D1_SEG) + (1 << E_SEG) + (1 << F_SEG) + (1 << A1_SEG) + (1 << A2_SEG) + (1 << B_SEG) + (1 << G2_SEG) + (1 << J_SEG)
#define DOLLAR    (1 << A2_SEG) + (1 << A1_SEG) + (1 << F_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << C_SEG) + (1 << D2_SEG) + (1 << D1_SEG) + (1 << J_SEG) + (1 << M_SEG)
#define PERCENT   (1 << A1_SEG) + (1 << F_SEG) + (1 << G1_SEG) + (1 << J_SEG) + (1 << G2_SEG) + (1 << M_SEG) + (1 << D2_SEG) + (1 << C_SEG) + (1 << N_SEG) + (1 << K_SEG)
#define CARROT    (1 << K_SEG) + (1 << N_SEG) + (1 << A2_SEG) + (1 << B_SEG)
#define AMPERSAND (1 << A2_SEG) + (1 << J_SEG) + (1 << K_SEG) + (1 << N_SEG) + (1 << D1_SEG) + (1 << M_SEG) + (1 << D2_SEG) + (1 << C_SEG)
#define ASTERISK  (1 << J_SEG) + (1 << H_SEG) + (1 << G1_SEG) + (1 << N_SEG) + (1 << M_SEG) + (1 << L_SEG) + (1 << G2_SEG) + (1 << K_SEG)
#define LPAREN    (1 << K_SEG) + (1 << L_SEG)
#define RPAREN    (1 << H_SEG) + (1 << N_SEG)
#define MINUS     (1 << G1_SEG) + (1 << G2_SEG)
#define UNDERSCORE    (1 << D1_SEG) + (1 << D2_SEG)
#define PLUSYSIGN (1 << J_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << M_SEG)
#define EQUALS    (1 << G1_SEG) + (1 << G2_SEG) + (1 << D1_SEG) + (1 << D2_SEG)

#define PERIOD    (1 << G1_SEG) + (1 << M_SEG) + (1 << D1_SEG) + (1 << E_SEG)
#define COLON     (1 << J_SEG) + (1 << M_SEG)
#define SEMICOLON (1 << J_SEG) + (1 << N_SEG)
#define LARROW    (1 << K_SEG) + (1 << L_SEG)
#define RARROW    (1 << H_SEG) + (1 << N_SEG)
#define COMMA     (1 << N_SEG)
#define FSLASH    (1 << N_SEG) + (1 << K_SEG)
#define BSLASH    (1 << H_SEG) + (1 << L_SEG)
#define SINGLEQUOTE   (1 << K_SEG)
#define DOUBLEQUOTE   (1 << J_SEG) + (1 << F_SEG)
#define LBRACKET  (1 << A2_SEG) + (1 << J_SEG) + (1 << M_SEG) + (1 << D2_SEG)
#define RIBRACKET (1 << A1_SEG) + (1 << J_SEG) + (1 << M_SEG) + (1 << D1_SEG)
#define LECURLY   (1 << A1_SEG) + (1 << J_SEG) + (1 << M_SEG) + (1 << D1_SEG) + (1 << G2_SEG)
#define RICURLY   (1 << G1_SEG) + (1 << A2_SEG) + (1 << J_SEG) + (1 << M_SEG) + (1 << D2_SEG)
#define PIPE      (1 << J_SEG) + (1 << M_SEG)
#define TILDE     (1 << E_SEG) + (1 << G1_SEG) + (1 << G2_SEG) + (1 << B_SEG)
#define APOSTROPHE    (1 << H_SEG)
#define QUESTIONMARK  (1 << M_SEG) + (1 << G2_SEG) + (1 << B_SEG) + (1 << A2_SEG)
#define POUNDSIGN     0xFFFF

#endif /* __ALPHANUM_H__ */

