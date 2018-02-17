#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include "dobby_pwm.h"

void init_pwm_pru(){
      tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
      prussdrv_init();
      prussdrv_open(PRU_EVTOUT_0);
  	  prussdrv_pruintc_init(&pruss_intc_initdata);
  }


void stop_pwm_pru() {
      int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);
      printf("PRU program completed, event number %d.\n", n);
      prussdrv_pru_disable(PRU_NUM);
      prussdrv_exit ();
  }

void exec_dobby_pwm() {
      prussdrv_exec_program (PRU_NUM, "./../bin/pwm.bin");
  }

unsigned int convert_pwm_pulsewidth(unsigned int pulsewidth_in_us) {
        unsigned int loop_counter_for_pru = pulsewidth_in_us*PRU_LOOP_TIME_US;
        return loop_counter_for_pru;
  }

void write_pwm_pulsewidth(unsigned int channel, unsigned int pulsewidth_in_us) {
			unsigned int data = convert_pwm_pulsewidth(pulsewidth_in_us);
			prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, channel-1, &data, 4);
  }
