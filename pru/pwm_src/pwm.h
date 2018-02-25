#ifndef PWM_H
#define PWM_H

#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <string.h>

#define PWM_PRU 1
#define PWM_CHANNELS 6
#define PULSE_TO_PRU_CYCLES 10

static unsigned int* pruDRAM0_32int_ptr;

int initialize_pru();
int disable_pru();
int write_pwm_pulse_us(int channel, int us);

#endif
