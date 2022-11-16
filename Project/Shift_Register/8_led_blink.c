/* 
* ---------------------------------------
* Copyright (c) Sebastian Günther 2021  |
*                                       |    
* devcon@admantium.com                  |    
*                                       | 
* SPDX-License-Identifier: BSD-3-Clause | 
* ---------------------------------------
*/
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <pico/stdlib.h>
#include "rp2040_shift_register.h"

/*
Pinouts:
    ___   ___
QB  |  |_|  |   VCC
QC  |   7   |   QA
QD  |   5   |   SER
QE  |   H   |   OE
QF  |   C   |   RCLK
QG  |   9   |   SRCLK
QH  |   5   |   SRCLR
GND |   A   |   QH'
    ---------


74HC595     pico
-------     ----
VCC         3.3V
SER         GPIO 12
OE          GND
RCLK        GPIO 10
SRCLK       GPIO 11
SRCLR       3.3V
*/

void main()
{
  ShiftRegister col_1_reg = shift_register_new((PinConfig){
      .SERIAL_PIN = 12,
      .SHIFT_REGISTER_CLOCK_PIN = 10,
      .STORAGE_REGISTER_CLOCK_PIN = 11});

  ShiftRegister row_reg = shift_register_new((PinConfig){
      .SERIAL_PIN = 9,
      .SHIFT_REGISTER_CLOCK_PIN = 10,
      .STORAGE_REGISTER_CLOCK_PIN = 11});


  // shift_register_write_bitmask(&row_reg, 0b1111111);
  // shift_register_flush(&row_reg);
  // sleep_ms(1050);

  // shift_register_write_bitmask(&col_1_reg, 0b1111111);
  // shift_register_flush(&col_1_reg);
  // sleep_ms(1050);

  // while(true){
  //   //QE
  //   shift_register_write_bitmask(&row_reg, 0b0000000);
  //   shift_register_flush(&row_reg);
  //   sleep_ms(1050);


  //   shift_register_write_bitmask(&row_reg, 0b1111111);
  //   shift_register_flush(&row_reg);
  //   sleep_ms(1050);

  // }

  while(true){
    //shift_register_write_bitmask(&col_1_reg, 0b10101010);
    shift_register_write_bitmask(&col_1_reg, 0b11110000);
    shift_register_flush(&col_1_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_1_reg, 0b0000000);
    shift_register_flush(&col_1_reg);
    sleep_ms(1050);
  }


}
