/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 *  - GPIO 5 ---> PWM output
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1.h"



// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];
fix15 accel_angle, gyro_angle_delta, complementary_angle;

volatile int Kp_int;
volatile int Ki_int;
volatile int kd_int;

fix15 Kp = int2fix15(600);
fix15 Ki = int2fix15(7);
fix15 Kd = int2fix15(10000);
fix15 error, error_accumulation, angle_increment, prev_error, error_deriv;

fix15 desired_angle = float2fix15(0.5);
fix15 duty_cycle;
fix15 Imax = int2fix15(2000);

// #define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
// #define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
// #define fix2float15(a) ((float)(a)/32768.0)
// #define absfix15(a) abs(a) 
// #define int2fix15(a) ((fix15)(a << 15))
// #define fix2int15(a) ((int)(a >> 15))
// #define char2fix15(a) (fix15)(((fix15)(a)) << 15)
// #define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
// character array
char screentext[40];

// draw speed
int threshold = 10 ;

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// semaphore
static struct pt_sem vga_semaphore ;

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  25.0f
uint slice_num ;

// PWM duty cycle
volatile int control ;
volatile int old_control ;


static char Angeltext[40];

// Interrupt service routine
void on_pwm_wrap() {



    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    // Gather measurements
    mpu6050_read_raw(acceleration, gyro);

    // Accelerometer angle (degrees - 15.16 fixed point)
    accel_angle = multfix15(divfix(acceleration[0], acceleration[1]), oneeightyoverpi) ;
    accel_angle = -accel_angle;

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    gyro_angle_delta = multfix15(gyro[2], zeropt001) ;

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001);

    // Compute the error
    error = (desired_angle - complementary_angle) ;

    // Start with angle_increment = 0.0001
    if (error < 0) {
        desired_angle -= angle_increment ;
    }
    else {
        desired_angle += angle_increment ;
    }
    // Integrate the error
    error_accumulation += error ;

    // Clamp the integrated error (start with Imax = max_duty_cycle/2)
    if (error_accumulation>Imax) error_accumulation=Imax ;
    if (error_accumulation<(-Imax)) error_accumulation=-Imax ;


    error_deriv = (error - prev_error) ;
    prev_error = error ;
    
    // Compute duty cycle with P controller
    // duty_cycle = kp * error * 1500
    //duty_cycle = (Kp * error) + (Ki * error_accumulation) ;
    duty_cycle =  multfix15(Kp, error)   +  multfix15(Ki, error_accumulation) + multfix15(Kd, error_deriv);

    //write to PWM
    control = fix2int15(duty_cycle);


    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Update duty cycle
    if (control!=old_control) {
        old_control = control ;       
        if(control > 0){
            if(control >= 5000){
                control = 5000;
            }
            pwm_set_chan_level(slice_num, PWM_CHAN_B, control);
            pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);
        }else{
            if(control <= -5000){
                control = 5000;
            }
            pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
            pwm_set_chan_level(slice_num, PWM_CHAN_A, -control);
        }
        
    }

    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    // drawHLine(75, 430, 5, CYAN) ;
    // drawHLine(75, 355, 5, CYAN) ;
    // drawHLine(75, 280, 5, CYAN) ;
    // drawVLine(80, 280, 150, CYAN) ;
    // sprintf(screentext, "0") ;
    // setCursor(50, 350) ;
    // writeString(screentext) ;
    // sprintf(screentext, "+2") ;
    // setCursor(50, 280) ;
    // writeString(screentext) ;
    // sprintf(screentext, "-2") ;
    // setCursor(50, 425) ;
    // writeString(screentext) ;

    // Draw top plot
    // drawHLine(75, 230, 5, CYAN) ;
    // drawHLine(75, 155, 5, CYAN) ;
    // drawHLine(75, 80, 5, CYAN) ;
    // drawVLine(80, 80, 150, CYAN) ;
    // sprintf(screentext, "0") ;
    // setCursor(50, 150) ;
    // writeString(screentext) ;
    // sprintf(screentext, "+250") ;
    // setCursor(45, 75) ;
    // writeString(screentext) ;
    // sprintf(screentext, "-250") ;
    // setCursor(45, 225) ;
    // writeString(screentext) ;
    
    sprintf(screentext, "CYAN for complementary_angle") ;
    setCursor(50, 150) ;
    writeString(screentext) ;

     sprintf(screentext, "RED for control value") ;
    setCursor(50, 225) ;
    writeString(screentext) ;

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;

        // //Display on VGA
        // fillRect(250, 30, 176, 30, BLACK); // red box
        // sprintf(Angeltext, "%f", (float)fix2float15(complementary_angle)) ;
        // setCursor(250, 30) ;
        // setTextSize(2) ;
        // writeString(Angeltext) ;

        // fillRect(250, 30, 176, 30, BLACK); // red box
        // sprintf(Angeltext, "%f", (float)fix2float15(complementary_angle)) ;
        // setCursor(250, 30) ;
        // setTextSize(2) ;
        // writeString(Angeltext) ;

        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // //Draw bottom plot (multiply by 120 to scale from +/-2 to +/-250)
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[0])*120.0)-OldMin)/OldRange)), WHITE) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[1])*120.0)-OldMin)/OldRange)), RED) ;
            // drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[2])*120.0)-OldMin)/OldRange)), GREEN) ;

            drawPixel(xcoord, 330 - (int)(NewRange*((float)((fix2float15(complementary_angle)*5)-OldMin)/OldRange)), CYAN) ;
            drawPixel(xcoord, 330 - (int)(NewRange*((float)((control*0.008)-OldMin)/OldRange)), RED) ;

            
            fillRect(250, 30, 176, 30, BLACK); // red box
            sprintf(Angeltext, "%d", control) ;
            setCursor(250, 30) ;
            setTextSize(2) ;
            writeString(Angeltext) ;

            fillRect(250, 50, 176, 50, BLACK); // red box
            sprintf(Angeltext, "%f", (float)fix2float15(complementary_angle)) ;
            setCursor(250, 50) ;
            setTextSize(2) ;
            writeString(Angeltext) ;
            //drawPixel(xcoord, 330 - (int)(NewRange*((float)((control*0.05)-OldMin)/OldRange)), RED) ;
            //drawPixel(xcoord, 330 - (int)(NewRange*((float)((control*0.05)-OldMin)/OldRange)), RED) ;


            // //Draw top plot
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[0]))-OldMin)/OldRange)), WHITE) ;
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[1]))-OldMin)/OldRange)), RED) ;
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[2]))-OldMin)/OldRange)), GREEN) ;

            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static int test_in ;
    static float float_in ;
    while(1) {

        //================================
        //change duty cycle
        //================================
        // sprintf(pt_serial_out_buffer, "input a duty cycle (-5000-5000): ");
        // serial_write ;
        // // spawn a thread to do the non-blocking serial read
        // serial_read ;
        // // convert input string to number
        // sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        // if (test_in > 5000) continue ;
        // else if (test_in < -5000) continue ;
        // else control = test_in ;
        //================================


        //================================
        //change kp ki kd
        //================================
        sprintf(pt_serial_out_buffer, "input Kp, Ki, Kd: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        sscanf(pt_serial_in_buffer,"%d %d %d", &Kp_int, &Ki_int, &kd_int) ;

        Kp = int2fix15(Kp_int);
        Ki = int2fix15(Ki_int);
        Kd = int2fix15(kd_int);

        sprintf(pt_serial_out_buffer, "curKp: %d curKi: %d curKd: %d \n", Kp_int, Ki_int, kd_int);
        serial_write ;
        // sprintf(pt_serial_out_buffer, "input a command: ");
        // serial_write ;
        // // spawn a thread to do the non-blocking serial read
        // serial_read ;
        // // convert input string to number
        // sscanf(pt_serial_in_buffer,"%c", &classifier) ;

        // // num_independents = test_in ;
        // if (classifier=='t') {
        //     sprintf(pt_serial_out_buffer, "timestep: ");
        //     serial_write ;
        //     serial_read ;
        //     // convert input string to number
        //     sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        //     if (test_in > 0) {
        //         threshold = test_in ;
        //     }
        //}
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 3125);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
