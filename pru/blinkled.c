#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#define PRU_NUM 0   // using PRU0 for these examples

void main (void){
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_0);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program (PRU_NUM, "./blinkled.bin");
	int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);
	printf("EBB PRU program completed, event number %d.\n", n);
	prussdrv_pru_disable(PRU_NUM);
	prussdrv_exit ();
}
