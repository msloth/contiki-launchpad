// THIS PROGRAM IS PROVIDED "AS IS". TI MAKES NO WARRANTIES OR
// REPRESENTATIONS, EITHER EXPRESS, IMPLIED OR STATUTORY,
// INCLUDING ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
// COMPLETENESS OF RESPONSES, RESULTS AND LACK OF NEGLIGENCE.
// TI DISCLAIMS ANY WARRANTY OF TITLE, QUIET ENJOYMENT, QUIET
// POSSESSION, AND NON-INFRINGEMENT OF ANY THIRD PARTY
// INTELLECTUAL PROPERTY RIGHTS WITH REGARD TO THE PROGRAM OR
// YOUR USE OF THE PROGRAM.
//
// IN NO EVENT SHALL TI BE LIABLE FOR ANY SPECIAL, INCIDENTAL,
// CONSEQUENTIAL OR INDIRECT DAMAGES, HOWEVER CAUSED, ON ANY
// THEORY OF LIABILITY AND WHETHER OR NOT TI HAS BEEN ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGES, ARISING IN ANY WAY OUT
// OF THIS AGREEMENT, THE PROGRAM, OR YOUR USE OF THE PROGRAM.
// EXCLUDED DAMAGES INCLUDE, BUT ARE NOT LIMITED TO, COST OF
// REMOVAL OR REINSTALLATION, COMPUTER TIME, LABOR COSTS, LOSS
// OF GOODWILL, LOSS OF PROFITS, LOSS OF SAVINGS, OR LOSS OF
// USE OR INTERRUPTION OF BUSINESS. IN NO EVENT WILL TI'S
// AGGREGATE LIABILITY UNDER THIS AGREEMENT OR ARISING OUT OF
// YOUR USE OF THE PROGRAM EXCEED FIVE HUNDRED DOLLARS
// (U.S.$500).
//
// Unless otherwise stated, the Program written and copyrighted
// by Texas Instruments is distributed as "freeware".  You may,
// only under TI's copyright in the Program, use and modify the
// Program without any charge or restriction.  You may
// distribute to third parties, provided that you transfer a
// copy of this license to the third party and the third party
// agrees to these terms by its first use of the Program. You
// must reproduce the copyright notice and any other legend of
// ownership on each copy or partial copy, of the Program.
//
// You acknowledge and agree that the Program contains
// copyrighted material, trade secrets and other TI proprietary
// information and is protected by copyright laws,
// international copyright treaties, and trade secret laws, as
// well as other intellectual property laws.  To protect TI's
// rights in the Program, you agree not to decompile, reverse
// engineer, disassemble or otherwise translate any object code
// versions of the Program to a human-readable form.  You agree
// that in no event will you alter, remove or destroy any
// copyright notice included in the Program.  TI reserves all
// rights not specifically granted under this license. Except
// as specifically provided herein, nothing in this agreement
// shall be construed as conferring by implication, estoppel,
// or otherwise, upon you, any license or other right under any
// TI patents, copyrights or trade secrets.
//
// You may not use the Program in non-TI devices.
/*---------------------------------------------------------------------------*/
//   MSP430 USCI I2C Transmitter and Receiver (Slave Mode)
//
//  Description: This code configures the MSP430's USCI module as
//  I2C slave capable of transmitting and receiving bytes.
//
//                    Slave
//                    MSP430
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |                 |
//            |                 |
//            |         SDA/P1.6|------->
//            |         SCL/P1.7|------->
//
// NOTE: External pull-ups are needed for SDA & SCL
/*---------------------------------------------------------------------------*/
#include <contiki.h>
#include "i2c-slave.h"

#include <msp430.h>
#include <legacymsp430.h>
/*---------------------------------------------------------------------------*/
static start_stop_callback_t start_stop_condition_callback = NULL;
static tx_callback_t transmit_callback = NULL;
static rx_callback_t receive_callback = NULL;
/*---------------------------------------------------------------------------*/
void
i2c_slave_init(start_stop_callback_t start_stop_callback,
               tx_callback_t tx_callback,
               rx_callback_t rx_callback,
               uint8_t slave_address,
               uint8_t sda_pin,
               uint8_t scl_pin)
{
  P1OUT &= ~(sda_pin | scl_pin);               // 
  P1SEL |= sda_pin | scl_pin;               // Assign I2C pins to USCI_B0
  P1SEL2 |= sda_pin | scl_pin;              // Assign I2C pins to USCI_B0
  UCB0CTL1 |= UCSWRST;                      // Enable SW reset
  UCB0CTL0 = UCMODE_3 + UCSYNC;             // I2C Slave, synchronous mode, 7 bit address
  UCB0I2COA = slave_address;                // set own (slave) address
  UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation

  IE2 |= UCB0TXIE + UCB0RXIE;               // Enable TX interrupt
  UCB0I2CIE |= UCSTTIE | UCSTPIE;                     // Enable START and STOP condition interrupt

  start_stop_condition_callback = start_stop_callback;
  receive_callback = rx_callback;
  transmit_callback = tx_callback;
}
/*---------------------------------------------------------------------------*/
interrupt(USCIAB0TX_VECTOR) usci_i2c_data_isr(void)
{
  if(IFG2 & UCB0TXIFG) {
    /* get data to tx buffer == will be transmitted */
    /* flagged when tx buffer is empty */
    transmit_callback(&UCB0TXBUF);
  } else {
    /* flagged when we have received a full byte */
    receive_callback(UCB0RXBUF);
  }
}
/*---------------------------------------------------------------------------*/
interrupt(USCIAB0RX_VECTOR) usci_i2c_state_isr(void)
{
  /* clear start/stop condition/address match flag */
  int start = (UCB0STAT & UCSTTIFG) ? 1 : 0;
  UCB0STAT &= ~(UCSTTIFG | UCSTPIFG);
  start_stop_condition_callback(start);
}
/*---------------------------------------------------------------------------*/
