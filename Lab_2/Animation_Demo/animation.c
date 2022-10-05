
/**
 * Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration animates two balls bouncing about the screen.
 * Through a serial interface, the user can change the ball color.
 *
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *
 * RESOURCES USED
 *  - PIO state machines 0, 1, and 2 on PIO instance 0
 *  - DMA channels 0, 1
 *  - 153.6 kBytes of RAM (for pixel color data)
 *
 */

// Include the VGA grahics library
#include "vga_graphics.h"
// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include Pico libraries
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// === the fixed point macros ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))

// Wall detection
// #define hitBottom(b) (b>int2fix15(380))
// #define hitTop(b) (b<int2fix15(100))
// #define hitLeft(a) (a<int2fix15(100))
// #define hitRight(a) (a>int2fix15(540))
volatile int top = 100;
volatile int bottom = 380;
volatile int left = 100;
volatile int right = 540;

int flag_1 = 0; 
int flag_2 = 0; 
int flag_3 = 0; 

int flag = 0;

// number of boids
#define n 300
//#define n1 100
//#define n2 n-n1
//volatile int n = 300;
volatile int n1 = 100;

// hit function
bool hitBottom(fix15 b){
  if(b > int2fix15(bottom)){
    return true;
  }else{
    return false;
  }
}

bool hitTop(fix15 b){
  if(b < int2fix15(top)){
    return true;
  }else{
    return false;
  }
}

bool hitLeft(fix15 b){
  if(b < int2fix15(left)){
    return true;
  }else{
    return false;
  }
}

bool hitRight(fix15 b){
  if(b > int2fix15(right)){
    return true;
  }else{
    return false;
  }
}

//hit boundry function
bool hitBottomBoundary(fix15 b){
  if(b > int2fix15(bottom)){
    return true;
  }else{
    return false;
  }
}

bool hitTopBoundary(fix15 b){
  if(b < int2fix15(top)){
    return true;
  }else{
    return false;
  }
}

bool hitLeftBoundary(fix15 b){
  if(b < int2fix15(left)){
    return true;
  }else{
    return false;
  }
}

bool hitRightBoundary(fix15 b){
  if(b > int2fix15(right)){
    return true;
  }else{
    return false;
  }
}

//max and min
#define min(x, y) ((x) < (y)) ? (x) : (y)
#define max(x, y) ((x) > (y)) ? (x) : (y)

// uS per frame
#define FRAME_RATE 33000

// the color of the boid
char color = WHITE ;

// Boid on core 0
fix15 boid0_x ;
fix15 boid0_y ;
fix15 boid0_vx ;
fix15 boid0_vy ;

// Boid on core 1
fix15 boid1_x ;
fix15 boid1_y ;
fix15 boid1_vx ;
fix15 boid1_vy ;



//
fix15 boid_x[n];
fix15 boid_y[n];
//
fix15 boid_vx[n];
fix15 boid_vy[n];

//
int id[n];

//count boid
volatile unsigned int boid_count = 0;

//

//=======================================
// Boids algorithm varibale
//=======================================
fix15 turnfactor = float2fix15(0.2);
fix15 visualRange = int2fix15(40);
fix15 protectedRange = int2fix15(15);
fix15 centering_factor = float2fix15(0.0005);
fix15 avoidfactor = float2fix15(0.05);
fix15 matching_factor = float2fix15(0.05);
fix15 maxspeed = int2fix15(6);
fix15 minspeed = int2fix15(3);
fix15 maxbias = float2fix15(0.01);
fix15 bias_increment = float2fix15(0.00004);
//(user-changeable, or updated dynamically)
fix15 biasval_0 = float2fix15(0.001); 
fix15 biasval_1 = float2fix15(0.001); 

//=======================================

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  // Start in center of screen
  //*x = int2fix15(320) ;
  *x = int2fix15(rand() % (530 - 110 + 1) + 110);
  
  //*y = int2fix15(240) ;
  *y = int2fix15(rand() % (370 - 110 + 1) + 110);

  // printf("x:%d, y:%d \n", *x, *y);
  // Choose left or right
   if (direction) *vx = int2fix15(2) ;
   else *vx = int2fix15(-2) ;
  //vx in the range of (2-5)
  //if (direction) *vx = int2fix15(rand() % (5 - 2 + 1) + 2) ;
  //else *vx = int2fix15(-(rand() % (5 - 2 + 1) + 2)) ;
  
  // Moving down
  *vy = int2fix15(2);
  //vy in the range of (2-5)
  //*vy = int2fix15(rand() % (5 - 2 + 1) + 2);
  // printf("velocity vx: %d, vy: %d \n", *vx, *vy);

}

// Draw the boundaries
void drawArena() {
  //printf("%d", top);
  drawVLine(left, top, bottom-top, WHITE) ;
  drawVLine(right, top, bottom-top, WHITE) ;
  drawHLine(left, bottom, right-left, WHITE) ;
  drawHLine(left, top, right-left, WHITE) ;
}


// Detect wallstrikes, update velocity and position
void wallsAndEdges_0(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // //=======================================
  // // Boids algorithm 
  // //=======================================

  int i = 0;
  int j = 0;
  for(i = 0; i<n1; i++){
   

    fix15 xpos_avg = float2fix15(0);
    fix15 ypos_avg = float2fix15(0);
    fix15 xvel_avg = float2fix15(0);
    fix15 yvel_avg = float2fix15(0);

    fix15 neighboring_boids = float2fix15(0);
    fix15 close_dx = float2fix15(0);
    fix15 close_dy = float2fix15(0);

    for(j = 0; j<n1; j++){
      if(j == i){
        continue;
      }
      fix15 dx = boid_x[i] - boid_x[j];
      fix15 dy = boid_y[i] - boid_y[j];
      
      if(absfix15(dx) < visualRange && absfix15(dy) < visualRange){
        fix15 squared_distance = multfix15(dx, dx) + multfix15(dy, dy);

        if(squared_distance < protectedRange){
          close_dx += boid_x[i] - boid_x[j];
          close_dy += boid_y[i] - boid_y[j];
        }else if(squared_distance < visualRange){
          xpos_avg += boid_x[j];
          ypos_avg += boid_y[j];
          xvel_avg += boid_vx[j];
          yvel_avg += boid_vy[j];

          neighboring_boids += int2fix15(1);
        } 
      }
    }
    if(neighboring_boids > int2fix15(0)){
      //Divide accumulator variables by number of boids in visual range
      xpos_avg = divfix(xpos_avg, neighboring_boids);
      ypos_avg = divfix(ypos_avg, neighboring_boids);
      xvel_avg = divfix(xvel_avg, neighboring_boids);
      yvel_avg = divfix(yvel_avg, neighboring_boids);

      //Add the centering/matching contributions to velocity
      boid_vx[i] = (boid_vx[i] + 
                  multfix15((xpos_avg - boid_x[i]), centering_factor) + 
                  multfix15((xvel_avg - boid_vx[i]), matching_factor));

      boid_vy[i] = (boid_vy[i] + 
                  multfix15((ypos_avg - boid_y[i]), centering_factor) + 
                  multfix15((yvel_avg - boid_vy[i]), matching_factor));
    }
    // Add the avoidance contribution to velocity
    boid_vx[i] = boid_vx[i] + multfix15(close_dx, avoidfactor);
    boid_vy[i] = boid_vy[i] + multfix15(close_dy, avoidfactor);
        
    // If the boid is near an edge, make it turn by turnfactor
    //(this describes a box, will vary based on boundary conditions)
    if (hitTop(boid_y[i])){
        boid_vy[i] = boid_vy[i] + turnfactor;
    }
    if (hitRight(boid_x[i])){
        boid_vx[i] = boid_vx[i] - turnfactor;
    }
    if (hitLeft(boid_x[i])){
        boid_vx[i] = boid_vx[i] + turnfactor;
    }
    if (hitBottom(boid_y[i])){
        boid_vy[i] = boid_vy[i] - turnfactor;
    }

    // if(flag_2 == 1){

    //   if (hitTopBoundary(boid_y[i])){
    //     boid_y[i] = int2fix15(bottom-1);
    //       //boid_vy[i] = boid_vy[i] + turnfactor;
    //   }
    //   if (hitBottomBoundary(boid_y[i])){
    //     boid_y[i] = int2fix15(top+1);
    //       //boid_vy[i] = boid_vy[i] - turnfactor;
    //   }

    //   if(flag_3 == 1){
    //     if (hitRightBoundary(boid_x[i])){
    //       boid_x[i] = int2fix15(left+1);
    //         //boid_vx[i] = boid_vx[i] - turnfactor;
    //     }
    //     if (hitLeftBoundary(boid_x[i])){
    //       boid_x[i] = int2fix15(right-1);
    //         //boid_vx[i] = boid_vx[i] + turnfactor;
    //     }
    //   }

    // }


    //##############################################################
    //### ECE 5730 students only - dynamically update bias value ###
    //##############################################################
    //# biased to right of screen

    //if(i<=n/2){

    if(boid_vx[i] > int2fix15(0)){
      biasval_0 = min(maxbias, biasval_0 + bias_increment);
    }else{
      biasval_0 = max(bias_increment, biasval_0 - bias_increment);
    }

    // }else{

    //   if(boid_vx[i] < int2fix15(0)){
    //     biasval_1 = min(maxbias, biasval_1 + bias_increment);
    //   }else{
    //     biasval_1 = max(bias_increment, biasval_1 - bias_increment);
    //   }
    // }

    //##############################################################
    //# If the boid has a bias, bias it!
    //# biased to right of screen

    //if(i<=n/2){
    boid_vx[i] = multfix15((int2fix15(1) - biasval_0), boid_vx[i]) + multfix15(biasval_0, int2fix15(1));
    // }else{
    // //# biased to left of screen
    //   boid_vx[i] = multfix15((int2fix15(1) - biasval_1), boid_vx[i]) + multfix15(biasval_1, int2fix15(-1));
    // }

    //# Calculate the boid's speed
    //# Slow step! Lookup the "alpha max plus beta min" algorithm

    // float temp1 = fix2float15(*vx);
    // float temp2 = fix2float15(*vy);
    // fix15 speed =  float2fix15(sqrt(temp1*temp1 + temp2 * temp2));

    float temp1 = fix2float15(boid_vx[i]);
    float temp2 = fix2float15(boid_vy[i]);

    // fix15 boid_vx_2 = multfix15(boid_vx[i], boid_vx[i]);
    // fix15 boid_vy_2 = multfix15(boid_vy[i], boid_vy[i]);

    fix15 speed = float2fix15(sqrt(temp1*temp1 + temp2 * temp2));

    //# Enforce min and max speeds
    if(speed < minspeed){
        boid_vx[i] = multfix15(divfix(boid_vx[i], speed), minspeed);
        boid_vy[i] = multfix15(divfix(boid_vy[i], speed), minspeed);
    }
    if(speed > maxspeed){
        boid_vx[i] = multfix15(divfix(boid_vx[i], speed), maxspeed);
        boid_vy[i] = multfix15(divfix(boid_vx[i], speed), maxspeed);
    }

    // //# Update boid's position
    // boid_x[i] = boid_x[i] + boid_vx[i];
    // boid_y[i] = boid_y[i] + boid_vy[i];
  }
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges_1(fix15* x, fix15* y, fix15* vx, fix15* vy)
{
  // //=======================================
  // // Boids algorithm 
  // //=======================================

  int i = 0;
  int j = 0;
  for(i = n1; i<n; i++){
   

    fix15 xpos_avg = float2fix15(0);
    fix15 ypos_avg = float2fix15(0);
    fix15 xvel_avg = float2fix15(0);
    fix15 yvel_avg = float2fix15(0);

    fix15 neighboring_boids = float2fix15(0);
    fix15 close_dx = float2fix15(0);
    fix15 close_dy = float2fix15(0);

    for(j = n1; j<n; j++){
      if(j == i){
        continue;
      }
      fix15 dx = boid_x[i] - boid_x[j];
      fix15 dy = boid_y[i] - boid_y[j];
      
      if(absfix15(dx) < visualRange && absfix15(dy) < visualRange){
        fix15 squared_distance = multfix15(dx, dx) + multfix15(dy, dy);

        if(squared_distance < protectedRange){
          close_dx += boid_x[i] - boid_x[j];
          close_dy += boid_y[i] - boid_y[j];
        }else if(squared_distance < visualRange){
          xpos_avg += boid_x[j];
          ypos_avg += boid_y[j];
          xvel_avg += boid_vx[j];
          yvel_avg += boid_vy[j];

          neighboring_boids += int2fix15(1);
        } 
      }
    }
    if(neighboring_boids > int2fix15(0)){
      //Divide accumulator variables by number of boids in visual range
      xpos_avg = divfix(xpos_avg, neighboring_boids);
      ypos_avg = divfix(ypos_avg, neighboring_boids);
      xvel_avg = divfix(xvel_avg, neighboring_boids);
      yvel_avg = divfix(yvel_avg, neighboring_boids);

      //Add the centering/matching contributions to velocity
      boid_vx[i] = (boid_vx[i] + 
                  multfix15((xpos_avg - boid_x[i]), centering_factor) + 
                  multfix15((xvel_avg - boid_vx[i]), matching_factor));

      boid_vy[i] = (boid_vy[i] + 
                  multfix15((ypos_avg - boid_y[i]), centering_factor) + 
                  multfix15((yvel_avg - boid_vy[i]), matching_factor));
    }
    // Add the avoidance contribution to velocity
    boid_vx[i] = boid_vx[i] + multfix15(close_dx, avoidfactor);
    boid_vy[i] = boid_vy[i] + multfix15(close_dy, avoidfactor);
        
    // If the boid is near an edge, make it turn by turnfactor
    //(this describes a box, will vary based on boundary conditions)
    if (hitTop(boid_y[i])){
        boid_vy[i] = boid_vy[i] + turnfactor;
    }
    if (hitRight(boid_x[i])){
        boid_vx[i] = boid_vx[i] - turnfactor;
    }
    if (hitLeft(boid_x[i])){
        boid_vx[i] = boid_vx[i] + turnfactor;
    }
    if (hitBottom(boid_y[i])){
        boid_vy[i] = boid_vy[i] - turnfactor;
    }

    // if(flag_2 == 1){

    //   if (hitTopBoundary(boid_y[i])){
    //     boid_y[i] = int2fix15(bottom-1);
    //       //boid_vy[i] = boid_vy[i] + turnfactor;
    //   }
    //   if (hitBottomBoundary(boid_y[i])){
    //     boid_y[i] = int2fix15(top+1);
    //       //boid_vy[i] = boid_vy[i] - turnfactor;
    //   }

    //   if(flag_3 == 1){
    //     if (hitRightBoundary(boid_x[i])){
    //       boid_x[i] = int2fix15(left+1);
    //         //boid_vx[i] = boid_vx[i] - turnfactor;
    //     }
    //     if (hitLeftBoundary(boid_x[i])){
    //       boid_x[i] = int2fix15(right-1);
    //         //boid_vx[i] = boid_vx[i] + turnfactor;
    //     }
    //   }

    // }


    //##############################################################
    //### ECE 5730 students only - dynamically update bias value ###
    //##############################################################
    //# biased to right of screen

    // if(i<=n/2){

    //   if(boid_vx[i] > int2fix15(0)){
    //     biasval_0 = min(maxbias, biasval_0 + bias_increment);
    //   }else{
    //     biasval_0 = max(bias_increment, biasval_0 - bias_increment);
    //   }

    // }else{

    if(boid_vx[i] < int2fix15(0)){
      biasval_1 = min(maxbias, biasval_1 + bias_increment);
    }else{
      biasval_1 = max(bias_increment, biasval_1 - bias_increment);
    }
    //}

    //##############################################################
    //# If the boid has a bias, bias it!
    //# biased to right of screen

    // if(i<=n/2){
    //   boid_vx[i] = multfix15((int2fix15(1) - biasval_0), boid_vx[i]) + multfix15(biasval_0, int2fix15(1));
    // }else{
    //# biased to left of screen
    boid_vx[i] = multfix15((int2fix15(1) - biasval_1), boid_vx[i]) + multfix15(biasval_1, int2fix15(-1));
    //}

    //# Calculate the boid's speed
    //# Slow step! Lookup the "alpha max plus beta min" algorithm

    // float temp1 = fix2float15(*vx);
    // float temp2 = fix2float15(*vy);
    // fix15 speed =  float2fix15(sqrt(temp1*temp1 + temp2 * temp2));

    float temp1 = fix2float15(boid_vx[i]);
    float temp2 = fix2float15(boid_vy[i]);

    // fix15 boid_vx_2 = multfix15(boid_vx[i], boid_vx[i]);
    // fix15 boid_vy_2 = multfix15(boid_vy[i], boid_vy[i]);

    fix15 speed = float2fix15(sqrt(temp1*temp1 + temp2 * temp2));

    //# Enforce min and max speeds
    if(speed < minspeed){
        boid_vx[i] = multfix15(divfix(boid_vx[i], speed), minspeed);
        boid_vy[i] = multfix15(divfix(boid_vy[i], speed), minspeed);
    }
    if(speed > maxspeed){
        boid_vx[i] = multfix15(divfix(boid_vx[i], speed), maxspeed);
        boid_vy[i] = multfix15(divfix(boid_vx[i], speed), maxspeed);
    }

    // //# Update boid's position
    // boid_x[i] = boid_x[i] + boid_vx[i];
    // boid_y[i] = boid_y[i] + boid_vy[i];
  }
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    static float biaval_float_0;
    static float biaval_float_1;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        
        // change Top value
        // print prompt

        //Mode 1 for box
        //Mode 2 for box with traverse
        //sprintf(pt_serial_out_buffer, "Mode, 1 for box\n 2 for box with top/bottom wrapping\n 3 for box with top/bottom left/right wrapping\n");
        //serial_write ;

        sprintf(pt_serial_out_buffer, "input the area [flag Top Bottom Left Right biasval0 biasval1]\n");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d %d %d %d %d %f %f %d", &flag, &top, &bottom, &left, &right, &biaval_float_0, &biaval_float_1, &n1);
        biasval_0 = float2fix15(biaval_float_0);
        biasval_1 = float2fix15(biaval_float_1);
        //sprintf(%f)
        if(flag == 1){
          flag_1 = 1;
          flag_2 = 0;
          flag_3 = 0;
        }else if(flag == 2){
          flag_1 = 0;
          flag_2 = 1;
          flag_3 = 0;
        }else if(flag == 3){
          flag_1 = 0;
          flag_2 = 1;
          flag_3 = 1;
        }

        sprintf(pt_serial_out_buffer, "biasval 0: %f biasval 1: %f \n", biaval_float_0, biaval_float_1);
        //sprintf(pt_serial_out_buffer, "flag: %d, Top: %d, Bottom:% d Left:%d right: %d\n", top, bottom, left, right);
        serial_write ;
        

        fillRect(0, 0, 640, 480, BLACK);


      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// Animation on core 0
static PT_THREAD (protothread_anim(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;


    //output on VGA
    // Write some text to VGA
    static char timetext[40];
    static int start_time;
    start_time = time_us_32() ; 
    static int duration_time;

    //count boid
    boid_count++;

    // Spawn a boid
    int i = 0;
    for(i = 0; i<n1; i++){
      spawnBoid(&boid_x[i], &boid_y[i], &boid_vx[i], &boid_vy[i], 0);
    }

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      // erase boid

      for(i = 0; i<n1; i++){
        drawRect(fix2int15(boid_x[i]), fix2int15(boid_y[i]), 2, 2, BLACK);
      }
      // update boid's position and velocity
      wallsAndEdges_0(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy) ;
      // draw the boid at its new position


      // //# Update boid's position
      for(i = 0; i<n1; i++){

        boid_x[i] = boid_x[i] + boid_vx[i];
        boid_y[i] = boid_y[i] + boid_vy[i];

        if(flag_2 == 1){

          if (hitTopBoundary(boid_y[i])){
            boid_y[i] = int2fix15(bottom-1);
              //boid_vy[i] = boid_vy[i] + turnfactor;
          }
          if (hitBottomBoundary(boid_y[i])){
            boid_y[i] = int2fix15(top+1);
              //boid_vy[i] = boid_vy[i] - turnfactor;
          }

          if(flag_3 == 1){
            if (hitRightBoundary(boid_x[i])){
              boid_x[i] = int2fix15(left+1);
                //boid_vx[i] = boid_vx[i] - turnfactor;
            }
            if (hitLeftBoundary(boid_x[i])){
              boid_x[i] = int2fix15(right-1);
                //boid_vx[i] = boid_vx[i] + turnfactor;
            }
          }
        }

        drawRect(fix2int15(boid_x[i]), fix2int15(boid_y[i]), 2, 2, color);
      }

      // for(i = 0; i<n; i++){
      //   drawRect(fix2int15(boid_x[i]), fix2int15(boid_y[i]), 2, 2, color);
      // } 
      // draw the boundaries
      drawArena() ;
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while

      //VGA output
      // duration_time = time_us_32()-start_time;
      // setTextColor(WHITE) ;
      // setCursor(65, 0) ;
      // setTextSize(2) ;
      // writeString("Time: ") ;
      // sprintf(timetext, "%d", (int)duration_time);
      // writeString(timetext);

      // //print to the terminal
      // printf("Time: %f\n",(float)duration_time/1000000.0);
      // printf("Boid numbers: %d\n", boid_count);
      // printf("FRAME_RATE: %d\n", FRAME_RATE);
    } // END WHILE(1)
  PT_END(pt);
} // animation thread


// Animation on core 1
static PT_THREAD (protothread_anim1(struct pt *pt))
{
    // Mark beginning of thread
    PT_BEGIN(pt);

    // Variables for maintaining frame rate
    static int begin_time ;
    static int spare_time ;


    //output on VGA
    // Write some text to VGA
    static char timetext[40];
    static int start_time;
    start_time = time_us_32() ; 
    static int duration_time;

    //count boid
    boid_count++;

    // Spawn a boid
    int i = 0;
    for(i = n1; i<n; i++){
      spawnBoid(&boid_x[i], &boid_y[i], &boid_vx[i], &boid_vy[i], 0);
    }

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      // erase boid

      for(i = n1; i<n; i++){
        drawRect(fix2int15(boid_x[i]), fix2int15(boid_y[i]), 2, 2, BLACK);
      }
      // update boid's position and velocity
      wallsAndEdges_1(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy) ;
      // draw the boid at its new position

      // //# Update boid's position
      for(i = n1; i<n; i++){


        boid_x[i] = boid_x[i] + boid_vx[i];
        boid_y[i] = boid_y[i] + boid_vy[i];

        
        if(flag_2 == 1){

          if (hitTopBoundary(boid_y[i])){
            boid_y[i] = int2fix15(bottom-1);
              //boid_vy[i] = boid_vy[i] + turnfactor;
          }
          if (hitBottomBoundary(boid_y[i])){
            boid_y[i] = int2fix15(top+1);
              //boid_vy[i] = boid_vy[i] - turnfactor;
          }

          if(flag_3 == 1){
            if (hitRightBoundary(boid_x[i])){
              boid_x[i] = int2fix15(left+1);
                //boid_vx[i] = boid_vx[i] - turnfactor;
            }
            if (hitLeftBoundary(boid_x[i])){
              boid_x[i] = int2fix15(right-1);
                //boid_vx[i] = boid_vx[i] + turnfactor;
            }
          }
        }

        drawRect(fix2int15(boid_x[i]), fix2int15(boid_y[i]), 2, 2, RED);
      }
      // draw the boundaries
      drawArena() ;
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while

      //VGA output
      // duration_time = time_us_32()-start_time;
      // setTextColor(WHITE) ;
      // setCursor(65, 0) ;
      // setTextSize(2) ;
      // writeString("Time: ") ;
      // sprintf(timetext, "%d", (int)duration_time);
      // writeString(timetext);

      // //print to the terminal
      // printf("Time: %f\n",(float)duration_time/1000000.0);
      // printf("Boid numbers: %d\n", boid_count);
      // printf("FRAME_RATE: %d\n", FRAME_RATE);
    } // END WHILE(1)
  PT_END(pt);
} // animation thread

// ========================================
// === core 1 main -- started in main below
// ========================================
void core1_main(){
  // Add animation thread
  pt_add_thread(protothread_anim1);
  // Start the scheduler
  pt_schedule_start ;

}

// ========================================
// === main
// ========================================
// USE ONLY C-sdk library
int main(){
  // initialize stio
  stdio_init_all() ;

  // initialize VGA
  initVGA() ;

  // start core 1 
  multicore_reset_core1();
  multicore_launch_core1(&core1_main);

  // add threads
  pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
