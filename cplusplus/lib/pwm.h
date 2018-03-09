#ifndef PWM_H
#define PWM_H

#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <iostream>
#include <stdint.h>
#include <chrono>
#include <thread>

#define PWM_PRU 1
#define PWM_CHANNELS 6
#define PULSE_TO_PRU_CYCLES 100/3.482
#define PWM_PERIOD 20000

#define ESC_LOW 1000
#define ESC_HIGH 2000
#define MOTOR_SPOOL_RATE 1100

typedef uint32_t u32;


struct channelPtr {
    u32 ch1;
    u32 ch2;
    u32 ch3;
    u32 ch4;
    u32 pwm_period;
  };

class Motors {
   private:
   channelPtr p;
   channelPtr volatile *channels;
   Receiver *recv;

   // sets the motors to spool rate (used after dobby is armed)
   void set_motors_spool_rate();

   public:
    int initialize_pru();
    int disable_pru();
    int update();
    int calibrate_esc();
    int arm_motors();

    // this will be accessed by smc controller, loading it
    // with latest PWM values
    u32 channel_val[4];

    bool is_pru_initialized;
    bool is_armed;

	  Motors(Receiver *recv_ptr);
};



#endif
