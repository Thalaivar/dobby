#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_NUM   1
#define PRU_LOOP_TIME_US 10*(0.001)

#define REG_OFFSET 32

bool init_pwm_pru();
void stop_pwm_pru();
void exec_dobby_pwm();
void write_pwm_pulsewidth(unsigned int channel_1, unsigned int channel_2, unsigned int channel_3, unsigned int channel_4);
unsigned int convert_pwm_pulsewidth(unsigned int pulsewidth_in_us);
