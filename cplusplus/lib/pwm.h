#ifndef PWM_H
#define PWM_H

#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <iostream>
#include <stdint.h>
#include <chrono>
#include <thread>
#include "ppm.h"

#define PWM_PRU 1
#define PWM_CHANNELS 6
#define PULSE_TO_PRU_CYCLES 100/3.482
#define PWM_PERIOD 20000

#define ESC_LOW 1000
#define ESC_HIGH 2000
#define MOTOR_SPOOL_RATE 1100

#define MOM_COEFF 0.169
#define THRUST_COEFF 2.724
#define DRAG_COEFF 1
#define THRUST_CONST 967
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

    // to send latest pulse to motors
    int update();

    // for calibrating esc when required
    int calibrate_esc();

    // checks if user gives signal to arm, returns -1 if signal was not to arm
    int arm_motors();

    // sends 1000us pulse to all 4 motors
    int disable_motors();

    // to get desired pulses from torques
    void demux_torques_to_pwm();

    // this will be accessed by smc controller to set desired torques
    float torques[3];

    // holds the pwm values to be sent
    int channel_val[4];

    bool is_pru_initialized;
    bool is_armed;

	  Motors(Receiver *recv_ptr);
};



#endif
