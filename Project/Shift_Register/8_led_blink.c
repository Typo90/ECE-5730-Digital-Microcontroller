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

// void main()
// {
//   ShiftRegister reg = shift_register_new((PinConfig){
//       .SERIAL_PIN = 9,
//       .SHIFT_REGISTER_CLOCK_PIN = 11,
//       .STORAGE_REGISTER_CLOCK_PIN = 10});

//   int switch_on = 0;

//   while (true)
//   {
//     shift_register_write_bit(&reg, 1);
//     shift_register_flush(&reg);

//     sleep_ms(1050);
//     switch_on++;

//     if (switch_on = 8)
//     {
//       shift_register_reset_storage(&reg);
//       sleep_ms(1050);
//       switch_on = 0;
//     }
//   }
// }

#include "shift_register_74hc595.h"

// Define shift register pins
#define ROW_1 12
#define COl 9
#define CLK_PIN 11
#define LATCH_PIN 10

int main() {
    // Set clk pin as output
    gpio_init(CLK_PIN);
    gpio_set_dir(CLK_PIN, GPIO_OUT);
    
    // Set data pin as output
    gpio_init(ROW_1);
    gpio_set_dir(ROW_1, GPIO_OUT);

    // Set latch pin as output
    gpio_init(LATCH_PIN);
    gpio_set_dir(LATCH_PIN, GPIO_OUT);

    // Create new shift register
    struct shift_register_74hc595_t *myreg = new_shreg_74hc595(CLK_PIN, ROW_1, LATCH_PIN);
    
    while (1) {
        for (size_t qi = QA; qi <= QH; qi++) {
            shreg_74hc595_put(myreg, qi, 1);
            sleep_ms(500);
        }
        for (size_t qi = QA; qi <= QH; qi++) {
            shreg_74hc595_put(myreg, qi, 0);
            sleep_ms(500);
        }
    }
}