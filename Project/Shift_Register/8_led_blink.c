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


#define ROW_1 0
#define ROW_2 1
#define ROW_3 2
#define ROW_4 3
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
ShiftRegister row_reg;
ShiftRegister col_1_reg;
ShiftRegister col_2_reg;
ShiftRegister col_3_reg;
ShiftRegister col_4_reg;


void testRow(){


  u_int8_t col_val = 0b11110000;

  shift_register_write_bitmask(&col_1_reg, col_val);
  shift_register_flush(&col_1_reg);


  shift_register_write_bitmask(&col_2_reg, col_val);
  shift_register_flush(&col_2_reg);


  shift_register_write_bitmask(&col_3_reg, col_val);
  shift_register_flush(&col_3_reg);


  shift_register_write_bitmask(&col_4_reg, col_val);
  shift_register_flush(&col_2_reg);


  gpio_put(ROW_1, 1);
  gpio_put(ROW_2, 1);
  gpio_put(ROW_3, 1);
  gpio_put(ROW_4, 0);

}


void drawOnePointRow(int z){
  if(z == 0){
    gpio_put(ROW_1, 0);
    gpio_put(ROW_2, 1);
    gpio_put(ROW_3, 1);
    gpio_put(ROW_4, 1);
  }else if(z == 1){
    gpio_put(ROW_1, 1);
    gpio_put(ROW_2, 0);
    gpio_put(ROW_3, 1);
    gpio_put(ROW_4, 1);
  }else if(z == 2){
    gpio_put(ROW_1, 1);
    gpio_put(ROW_2, 1);
    gpio_put(ROW_3, 0);
    gpio_put(ROW_4, 1);
  }else if(z == 3){
    gpio_put(ROW_1, 1);
    gpio_put(ROW_2, 1);
    gpio_put(ROW_3, 1);
    gpio_put(ROW_4, 0);
  }
}

void lineByLine(){

    // row to ALL ground
    shift_register_write_bitmask(&row_reg, 0b00000000);
    shift_register_flush(&row_reg);
    

    // reg 4
    shift_register_write_bitmask(&col_4_reg, 0b10000000);
    shift_register_flush(&col_4_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_4_reg, 0b01000000);
    shift_register_flush(&col_4_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_4_reg, 0b00100000);
    shift_register_flush(&col_4_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_4_reg, 0b00010000);
    shift_register_flush(&col_4_reg);
    sleep_ms(1050);

    // reg 3 
    shift_register_write_bitmask(&row_reg, 0b00000000);
    shift_register_flush(&row_reg);
    sleep_ms(1050);
    
    shift_register_write_bitmask(&col_3_reg, 0b10000000);
    shift_register_flush(&col_3_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_3_reg, 0b01000000);
    shift_register_flush(&col_3_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_3_reg, 0b00100000);
    shift_register_flush(&col_3_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_3_reg, 0b00010000);
    shift_register_flush(&col_3_reg);
    sleep_ms(1050);
    
    // reg 2
    shift_register_write_bitmask(&row_reg, 0b00000000);
    shift_register_flush(&row_reg);
    sleep_ms(1050);
    
    shift_register_write_bitmask(&col_2_reg, 0b10000000);
    shift_register_flush(&col_2_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_2_reg, 0b01000000);
    shift_register_flush(&col_2_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_2_reg, 0b00100000);
    shift_register_flush(&col_2_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_2_reg, 0b00010000);
    shift_register_flush(&col_2_reg);
    sleep_ms(1050);

    // reg 1
    shift_register_write_bitmask(&row_reg, 0b00000000);
    shift_register_flush(&row_reg);
    sleep_ms(1050);
    
    shift_register_write_bitmask(&col_1_reg, 0b10000000);
    shift_register_flush(&col_1_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_1_reg, 0b01000000);
    shift_register_flush(&col_1_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_1_reg, 0b00100000);
    shift_register_flush(&col_1_reg);
    sleep_ms(1050);

    shift_register_write_bitmask(&col_1_reg, 0b00010000);
    shift_register_flush(&col_1_reg);
    sleep_ms(1050);
}

void drawFFTLayer0(){


  u_int8_t col_val = 0b11110000;

  shift_register_write_bitmask(&col_1_reg, col_val);
  shift_register_flush(&col_1_reg);


  // shift_register_write_bitmask(&col_2_reg, col_val);
  // shift_register_flush(&col_2_reg);


  // shift_register_write_bitmask(&col_3_reg, col_val);
  // shift_register_flush(&col_3_reg);


  // shift_register_write_bitmask(&col_4_reg, col_val);
  // shift_register_flush(&col_2_reg);


  gpio_put(ROW_1, 1);
  gpio_put(ROW_2, 1);
  gpio_put(ROW_3, 1);
  gpio_put(ROW_4, 0);
    
}

void drawFFTLayer1(){


  u_int8_t col_val = 0b11110000;

  // shift_register_write_bitmask(&col_1_reg, col_val);
  // shift_register_flush(&col_1_reg);

  shift_register_write_bitmask(&col_2_reg, col_val);
  shift_register_flush(&col_2_reg);

  // shift_register_write_bitmask(&col_3_reg, col_val);
  // shift_register_flush(&col_3_reg);

  // shift_register_write_bitmask(&col_4_reg, col_val);
  // shift_register_flush(&col_4_reg);


  gpio_put(ROW_1, 1);
  gpio_put(ROW_2, 1);
  gpio_put(ROW_3, 0);
  gpio_put(ROW_4, 0);
}

void drawFFTLayer2(){


  u_int8_t col_val = 0b11110000;

  // shift_register_write_bitmask(&col_1_reg, col_val);
  // shift_register_flush(&col_1_reg);

  shift_register_write_bitmask(&col_2_reg, col_val);
  shift_register_flush(&col_2_reg);

  //shift_register_write_bitmask(&col_3_reg, col_val);
  //shift_register_flush(&col_3_reg);

  // shift_register_write_bitmask(&col_4_reg, col_val);
  // shift_register_flush(&col_4_reg);

  gpio_put(ROW_1, 1);
  gpio_put(ROW_2, 0);
  gpio_put(ROW_3, 0);
  gpio_put(ROW_4, 0);
}

void drawFFTLayer3(){


  u_int8_t col_val = 0b11110000;
  

  // shift_register_write_bitmask(&col_1_reg, col_val);
  // shift_register_flush(&col_1_reg);

  shift_register_write_bitmask(&col_2_reg, col_val);
  shift_register_flush(&col_2_reg);

  // shift_register_write_bitmask(&col_3_reg, col_val);
  // shift_register_flush(&col_3_reg);

  // shift_register_write_bitmask(&col_4_reg, col_val);
  // shift_register_flush(&col_4_reg);

  gpio_put(ROW_1, 0);
  gpio_put(ROW_2, 0);
  gpio_put(ROW_3, 0);
  gpio_put(ROW_4, 0);
}

void drawFFTLayer4(){


  u_int8_t col_val = 0b11110000;
  

  // shift_register_write_bitmask(&col_1_reg, col_val);
  // shift_register_flush(&col_1_reg);

  // shift_register_write_bitmask(&col_2_reg, col_val);
  // shift_register_flush(&col_2_reg);

  shift_register_write_bitmask(&col_3_reg, col_val);
  shift_register_flush(&col_3_reg);

  // shift_register_write_bitmask(&col_4_reg, col_val);
  // shift_register_flush(&col_4_reg);

  gpio_put(ROW_1, 1);
  gpio_put(ROW_2, 0);
  gpio_put(ROW_3, 0);
  gpio_put(ROW_4, 0);
}

void drawFFTLayer5(){


  u_int8_t col_val = 0b11110000;
  

  // shift_register_write_bitmask(&col_1_reg, col_val);
  // shift_register_flush(&col_1_reg);

  // shift_register_write_bitmask(&col_2_reg, col_val);
  // shift_register_flush(&col_2_reg);

  shift_register_write_bitmask(&col_3_reg, col_val);
  shift_register_flush(&col_3_reg);

  // shift_register_write_bitmask(&col_4_reg, col_val);
  // shift_register_flush(&col_4_reg);

  gpio_put(ROW_1, 0);
  gpio_put(ROW_2, 0);
  gpio_put(ROW_3, 0);
  gpio_put(ROW_4, 0);
}

void drawFFTLayer6(){


  u_int8_t col_val = 0b11110000;

  // shift_register_write_bitmask(&col_1_reg, col_val);
  // shift_register_flush(&col_1_reg);

  // shift_register_write_bitmask(&col_2_reg, col_val);
  // shift_register_flush(&col_2_reg);

  shift_register_write_bitmask(&col_3_reg, col_val);
  shift_register_flush(&col_3_reg);

  // shift_register_write_bitmask(&col_4_reg, col_val);
  // shift_register_flush(&col_4_reg);


  gpio_put(ROW_1, 1);
  gpio_put(ROW_2, 1);
  gpio_put(ROW_3, 0);
  gpio_put(ROW_4, 0);
}

void drawFFTLayer7(){


  u_int8_t col_val = 0b11110000;

  // shift_register_write_bitmask(&col_1_reg, col_val);
  // shift_register_flush(&col_1_reg);


  // shift_register_write_bitmask(&col_2_reg, col_val);
  // shift_register_flush(&col_2_reg);


  // shift_register_write_bitmask(&col_3_reg, col_val);
  // shift_register_flush(&col_3_reg);


  shift_register_write_bitmask(&col_4_reg, col_val);
  shift_register_flush(&col_2_reg);


  gpio_put(ROW_1, 1);
  gpio_put(ROW_2, 1);
  gpio_put(ROW_3, 1);
  gpio_put(ROW_4, 0);
    
}


void drawFancy0(){
  u_int8_t col_val = 0b11110000;

  gpio_put(ROW_1, 0);
  gpio_put(ROW_2, 0);
  gpio_put(ROW_3, 0);
  gpio_put(ROW_4, 0);

  shift_register_write_bitmask(&col_1_reg, col_val);
  shift_register_flush(&col_1_reg);
  sleep_ms(500);

  shift_register_write_bitmask(&col_2_reg, col_val);
  shift_register_flush(&col_2_reg);
  sleep_ms(500);

  shift_register_write_bitmask(&col_3_reg, col_val);
  shift_register_flush(&col_3_reg);
  sleep_ms(500);

  shift_register_write_bitmask(&col_4_reg, col_val);
  shift_register_flush(&col_4_reg);
  sleep_ms(500);

  shift_register_write_bitmask(&col_3_reg, col_val);
  shift_register_flush(&col_3_reg);
  sleep_ms(500);

  shift_register_write_bitmask(&col_2_reg, col_val);
  shift_register_flush(&col_2_reg);
  sleep_ms(500);

  shift_register_write_bitmask(&col_1_reg, col_val);
  shift_register_flush(&col_1_reg);
  sleep_ms(500);
}

void drawOnePoint(int x, int y, int z){
    if(x > 4 || x < 0 || y > 4 || y < 0){
      return;
    }


    u_int8_t col_val = 0b10000000;

    // x == 0, col_val = 0b1000000;
    // x == 1, col_val = 0b0100000;
    // x == 2, col_val = 0b0010000;
    if(x == 0){
      col_val = 0b10000000;
    }else if(x == 1){
      col_val = 0b01000000;
    }else if(x == 2){
      col_val = 0b00100000;
    }else if(x == 3){
      col_val = 0b00010000;
    }

    if(y == 0){
      drawOnePointRow(z);
      shift_register_write_bitmask(&col_1_reg, col_val);
      shift_register_flush(&col_1_reg);
    }else if(y == 1){
      drawOnePointRow(z);
      shift_register_write_bitmask(&col_2_reg, col_val);
      shift_register_flush(&col_2_reg);
    }else if(y == 2){
      drawOnePointRow(z);
      shift_register_write_bitmask(&col_3_reg, col_val);
      shift_register_flush(&col_3_reg);
    }else if(y == 3){
      drawOnePointRow(z);
      shift_register_write_bitmask(&col_4_reg, col_val);
      shift_register_flush(&col_4_reg);
    }

}

void main()
{


  row_reg = shift_register_new((PinConfig){
      .SERIAL_PIN = 15,
      .SHIFT_REGISTER_CLOCK_PIN = 10,
      .STORAGE_REGISTER_CLOCK_PIN = 11});

  col_1_reg = shift_register_new((PinConfig){
      .SERIAL_PIN = 14,
      .SHIFT_REGISTER_CLOCK_PIN = 10,
      .STORAGE_REGISTER_CLOCK_PIN = 11});

  // col_2_reg = shift_register_new((PinConfig){
  //     .SERIAL_PIN = 13,
  //     .SHIFT_REGISTER_CLOCK_PIN = 10,
  //     .STORAGE_REGISTER_CLOCK_PIN = 11});

  col_2_reg = shift_register_new((PinConfig){
    .SERIAL_PIN = 13,
    .SHIFT_REGISTER_CLOCK_PIN = 8,
    .STORAGE_REGISTER_CLOCK_PIN = 11});

  col_3_reg = shift_register_new((PinConfig){
      .SERIAL_PIN = 12,
      .SHIFT_REGISTER_CLOCK_PIN = 10,
      .STORAGE_REGISTER_CLOCK_PIN = 11});

  col_4_reg = shift_register_new((PinConfig){
      .SERIAL_PIN = 9,
      .SHIFT_REGISTER_CLOCK_PIN = 10,
      .STORAGE_REGISTER_CLOCK_PIN = 11});


  gpio_init(ROW_1);
  gpio_set_dir(ROW_1, GPIO_OUT);

  gpio_init(ROW_2);
  gpio_set_dir(ROW_2, GPIO_OUT);

  gpio_init(ROW_3);
  gpio_set_dir(ROW_3, GPIO_OUT);

  gpio_init(ROW_4);
  gpio_set_dir(ROW_4, GPIO_OUT);


  drawFancy0();
  while(true){



    // for(int i = 0; i<4; i++){
    //   for(int j = 0; j<4; j++){
    //     for(int k = 0; k<4; k++){
    //       drawOnePoint(i, j, k);
    //       sleep_ms(250);
    //     }
    //   }
    // }



    // drawFFTLayer0();
    // sleep_ms(500);
    // drawFFTLayer1();
    // sleep_ms(250);
    // drawFFTLayer2();
    // sleep_ms(250);
    // drawFFTLayer3();
    // sleep_ms(250);
    // drawFFTLayer4();
    // sleep_ms(250);
    // drawFFTLayer5();
    // sleep_ms(250);
    // drawFFTLayer6();
    // sleep_ms(250);
    // drawFFTLayer7();
    // sleep_ms(500);


    testRow();

    //drawOnePoint(0, 0, 0);

    //lineByLine();

  }

}



