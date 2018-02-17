#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#define PRU_NUM 1   // using PRU0 for these examples

void main (void){
		tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
		prussdrv_init ();
		prussdrv_open (PRU_EVTOUT_0);

		prussdrv_pruintc_init(&pruss_intc_initdata);
		
		int data = [1000, 1200, 1100, 1300]
		prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 0, data, 16);

		prussdrv_exec_program (PRU_NUM, "../../bin/./echo.bin")
		

		int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);
