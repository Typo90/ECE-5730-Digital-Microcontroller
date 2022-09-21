
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
#define hitBottom(b) (b>int2fix15(380))
#define hitTop(b) (b<int2fix15(100))
#define hitLeft(a) (a<int2fix15(100))
#define hitRight(a) (a>int2fix15(540))

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
fix15 boid_x[10];
fix15 boid_y[10];
fix15 boid_vx[10];
fix15 boid_vy[10];

//
int id[10];
//count boid
volatile unsigned int boid_count = 0;
//=======================================
// Boids algorithm varibale
//=======================================
fix15 turnfactor = float2fix15(0.2);
fix15 visualRange = 40;
fix15 protectedRange = 8;
fix15 centering_factor = 0.0005;
fix15 avoidfactor = 0.05;
fix15 matching_factor = 0.05;
fix15 maxspeed = int2fix15(6);
fix15 minspeed = int2fix15(3);
fix15 maxbias = 0.01;
fix15 bias_increment = 0.00004;
//(user-changeable, or updated dynamically)
fix15 biasval_0 = 0.001; 
fix15 biasval_1 = 0.001; 

//=======================================

// Create a boid
void spawnBoid(fix15* x, fix15* y, fix15* vx, fix15* vy, int direction)
{
  // Start in center of screen
  //*x = int2fix15(320) ;
  *x = int2fix15(rand() % (530 - 110 + 1) + 110);
  
  //*y = int2fix15(240) ;
  *y = int2fix15(rand() % (370 - 110 + 1) + 110);

  printf("x:%d, y:%d \n", *x, *y);
  // Choose left or right
  // if (direction) *vx = int2fix15(3) ;
  // else *vx = int2fix15(-3) ;
  //vx in the range of (3-5)
  if (direction) *vx = int2fix15(rand() % (5 - 3 + 1) +3  ) ;
  else *vx = int2fix15(-(rand() % (5 - 3 + 1) + 3)) ;
  
  // Moving down
  //*vy = int2fix15(1) ;
  //vy in the range of (1-10)
  *vy = int2fix15(rand() % (10) + 1);
  printf("velocity vx: %d, vy: %d \n", *vx, *vy);
}

// Draw the boundaries
void drawArena() {
  drawVLine(100, 100, 280, WHITE) ;
  drawVLine(540, 100, 280, WHITE) ;
  drawHLine(100, 100, 440, WHITE) ;
  drawHLine(100, 380, 440, WHITE) ;
}

// Detect wallstrikes, update velocity and position
void wallsAndEdges(fix15* x, fix15* y, fix15* vx, fix15* vy)
{

  

  // //=======================================
  // // Boids algorithm 
  // //=======================================

  // fix15 xpos_avg = 0;
  // fix15 ypos_avg = 0;
  // fix15 xvel_avg = 0;
  // fix15 yvel_avg = 0;

  // int neighboring_boids = 0;
  // fix15 close_dx = 0;
  // fix15 close_dy = 0;

  // fix15 dx = boid0_x - boid1_x;
  // fix15 dy = boid0_y - boid1_y;

  // if(abs(dx) < visualRange && abs(dy) < visualRange){
  //   fix15 squared_distance = dx*dx + dy*dy;

  //   if(squared_distance < protectedRange){
  //     close_dx += boid0_x - boid1_x;
  //     close_dy += boid0_y - boid1_y;
  //   }else if(squared_distance < visualRange){
  //     xpos_avg += boid1_x;
  //     ypos_avg += boid1_y;
  //     xvel_avg += boid1_vx;
  //     yvel_avg += boid1_vy;

  //     neighboring_boids += 1;
  //   } 
  // }

  // if(neighboring_boids > 0){
  //   //Divide accumulator variables by number of boids in visual range
  //   xpos_avg = xpos_avg/neighboring_boids;
  //   ypos_avg = ypos_avg/neighboring_boids;
  //   xvel_avg = xvel_avg/neighboring_boids;
  //   yvel_avg = yvel_avg/neighboring_boids;

  //   //Add the centering/matching contributions to velocity
  //   boid0_vx = (boid0_vx + 
  //               (xpos_avg - boid0_x)*centering_factor + 
  //               (xvel_avg - boid0_vx)*matching_factor);

  //   boid0_vy = (boid0_vy + 
  //               (ypos_avg - boid0_y)*centering_factor + 
  //               (yvel_avg - boid0_vy)*matching_factor);
  // }
  // // Add the avoidance contribution to velocity
  // boid0_vx = boid0_vx + (close_dx*avoidfactor);
  // boid0_vy = boid0_vy + (close_dy*avoidfactor);
      
  // // If the boid is near an edge, make it turn by turnfactor
  // //(this describes a box, will vary based on boundary conditions)
  // if (hitTop(*y)){
  //     boid0_vy = boid0_vy + turnfactor;
  // }
  // if (hitRight(*x)){
  //     boid0_vx = boid0_vx - turnfactor;
  // }
  // if (hitLeft(*x)){
  //     boid0_vx = boid0_vx + turnfactor;
  // }
  // if (hitBottom(*y)){
  //     boid0_vy = boid0_vy - turnfactor;
  // }

  // //##############################################################
  // //### ECE 5730 students only - dynamically update bias value ###
  // //##############################################################
  // //# biased to right of screen
  // if(boid0_vx > 0){
  //   biasval_0 = min(maxbias, biasval_0 + bias_increment);
  // }else{
  //   biasval_0 = max(bias_increment, biasval_0 - bias_increment);
  // }

  // if(boid1_x < 0){
  //   biasval_1 = min(maxbias, biasval_1 + bias_increment);
  // }else{
  //   biasval_1 = max(bias_increment, biasval_1 - bias_increment);
  // }

  // //##############################################################
  // //# If the boid has a bias, bias it!
  // //# biased to right of screen

  //   boid0_vx = (1 - biasval_0)*boid0_vx + (biasval_0 * 1);
  // //# biased to left of screen
  //   boid1_vx = (1 - biasval_1)*boid1_vx + (biasval_1 * (-1));

  // //# Calculate the boid's speed
  // //# Slow step! Lookup the "alpha max plus beta min" algorithm
  // fix15 speed = sqrt(multfix15(*vx, *vx) + multfix15(*vy, *vy));

  // //# Enforce min and max speeds
  // if(speed < minspeed){
  //     *vx = (*vx/speed)*minspeed;
  //     *vy = (*vy/speed)*minspeed;
  // }
  // if(speed > maxspeed){
  //     *vx = (*vx/speed)*maxspeed;
  //     *vy = (*vy/speed)*maxspeed;
  // }

  // //# Update boid's position
  // *x = *x + *vx;
  // *y = *y + *vy;
  // // boid.x = boid.x + boid.vx
  // // boid.y = boid.y + boid.vy

    
  //   // if (boid in scout group 1): 
  //   //     if (boid.vx > 0):
  //   //         boid.biasval = min(maxbias, boid.biasval + bias_increment)
  //   //     else:
  //   //         boid.biasval = max(bias_increment, boid.biasval - bias_increment)
  //   //# biased to left of screen
  //   // else if (boid in scout group 2): # biased to left of screen
  //   //     if (boid.vx < 0):
  //   //         boid.biasval = min(maxbias, boid.biasval + bias_increment)
  //   //     else:
  //   //         boid.biasval = max(bias_increment, boid.biasval - bias_increment)
  //   //##############################################################
  
  

  // Reverse direction if we've hit a wall
  if (hitTop(*y)) {
    *vy = *vy + turnfactor ;
  }
  if (hitBottom(*y)) {
    *vy = *vy - turnfactor ;
  } 
  if (hitRight(*x)) {
    *vx = *vx - turnfactor;
  }
  if (hitLeft(*x)) {
    *vx = *vx + turnfactor;
  } 
  // if(*y >= 380){
  //   *vy = *vy - turnfactor ;
  // }
  // if(*y <= 100){
  //   *vy = *vy + turnfactor ;
  // }

  // if(*x >= 540){
  //   *vx = *vx - turnfactor;
  // }

  // if(*x <= 100){
  //   *vx = *vx + turnfactor;
  // }

  float temp1 = fix2float15(*vx);
  float temp2 = fix2float15(*vy);
  fix15 speed =  float2fix15(sqrt(temp1*temp1 + temp2 * temp2));


  if(speed < minspeed){
      *vx = multfix15(divfix(*vx,speed), minspeed);
      *vy = multfix15(divfix(*vy,speed), minspeed);
  }
  if(speed > maxspeed){
      *vx = multfix15(divfix(*vx, speed), maxspeed);
      *vy = multfix15(divfix(*vy, speed), maxspeed);
  }

  //printf("vx: %d, vy: %d \n", fix2int15(*vx),fix2int15(*vy));

  //Update position using velocity
  *x = *x + *vx ;
  *y = *y + *vy ;
}

// ==================================================
// === users serial input thread
// ==================================================
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt);
    // stores user input
    static int user_input ;
    // wait for 0.1 sec
    PT_YIELD_usec(1000000) ;
    // announce the threader version
    sprintf(pt_serial_out_buffer, "Protothreads RP2040 v1.0\n\r");
    // non-blocking write
    serial_write ;
      while(1) {
        // print prompt
        sprintf(pt_serial_out_buffer, "input a number in the range 1-7 099159451985: ");
        // non-blocking write
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d", &user_input) ;
        // update boid color
        if ((user_input > 0) && (user_input < 8)) {
          color = (char)user_input ;
        }
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
    spawnBoid(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy, 0);


    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      // erase boid

      drawRect(fix2int15(boid0_x), fix2int15(boid0_y), 2, 2, BLACK);
      // update boid's position and velocity
      wallsAndEdges(&boid0_x, &boid0_y, &boid0_vx, &boid0_vy) ;
      // draw the boid at its new position
      drawRect(fix2int15(boid0_x), fix2int15(boid0_y), 2, 2, color); 
      // draw the boundaries
      drawArena() ;
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while

      //VGA output
      duration_time = time_us_32()-start_time;
      setTextColor(WHITE) ;
      setCursor(65, 0) ;
      setTextSize(2) ;
      writeString("Time: ") ;
      sprintf(timetext, "%d", (int)duration_time);
      writeString(timetext);

      //print to the terminal
      printf("Time: %f\n",(float)duration_time/1000000.0);
      printf("Boid numbers: %d\n", boid_count);
      printf("FRAME_RATE: %d\n", FRAME_RATE);
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

    // Spawn a boid
    // with random location
    spawnBoid(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy, 1);

    //count boid
    boid_count++;

    while(1) {
      // Measure time at start of thread
      begin_time = time_us_32() ;      
      // erase boid
      drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, BLACK);
      // update boid's position and velocity
      wallsAndEdges(&boid1_x, &boid1_y, &boid1_vx, &boid1_vy) ;
      // draw the boid at its new position
      drawRect(fix2int15(boid1_x), fix2int15(boid1_y), 2, 2, color); 
      // delay in accordance with frame rate
      spare_time = FRAME_RATE - (time_us_32() - begin_time) ;
      // yield for necessary amount of time
      PT_YIELD_usec(spare_time) ;
     // NEVER exit while
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
  //multicore_reset_core1();
  //multicore_launch_core1(&core1_main);

  // add threads
  //pt_add_thread(protothread_serial);
  pt_add_thread(protothread_anim);

  // start scheduler
  pt_schedule_start ;
} 
