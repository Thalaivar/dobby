#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>

#define PRU_NUM   1

void main (void){
		tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
		prussdrv_init ();
		prussdrv_open (PRU_EVTOUT_0);
		prussdrv_pruintc_init(&pruss_intc_initdata);
		unsigned int pulsewidth =100000;
		prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 0, &pulsewidth, 4);
		prussdrv_exec_program (PRU_NUM, "./pwm.bin");
		int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);
		printf("PRU program completed, event number %d.\n", n);
		prussdrv_pru_disable(PRU_NUM);
		prussdrv_exit ();
	 }
