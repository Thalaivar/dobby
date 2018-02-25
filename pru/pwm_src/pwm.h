#ifndef PWM_H
#define PWM_H

#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <string.h>
#include <stdint.h>

#define PWM_PRU 1
#define PWM_CHANNELS 6
#define PULSE_TO_PRU_CYCLES 10
#define PWM_PERIOD 20000
#define ESC_LOW 1000
#define ESC_HIGH 2000

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

   public:
    int initialize_pru();
    int disable_pru();
    int update();
    int calibrate_esc();
    // this will be accessed by smc controller, loading it
    // with latest PWM values
    u32 channel_val[4];
    bool is_initialized;

	Motors();
};



#endif
