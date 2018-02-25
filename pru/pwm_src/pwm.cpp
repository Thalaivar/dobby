#include "pwm.h"

int initialize_pru(){
	
	
	// reset pwm  pointer to NULL so if init fails it doesn't point somewhere bad
	pruDRAM0_32int_ptr = NULL;
	
	// Initialise driver
	prussdrv_init ();
	
	// Open interrupt
	unsigned int ret = prussdrv_open(PRU_EVTOUT_0);
	if (ret) {
		printf("prussdrv_open open failed\n");
		return -1;
	}

	//Initialise interrupt
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;	
	prussdrv_pruintc_init(&pruss_intc_initdata);

	//// start mmaping dram memory
	printf("mmaping PRU1 DRAM memory\n");
	static void* memPointer;
	prussdrv_map_prumem(PRUSS0_PRU1_DATARAM, &memPointer);
	
	// set global  memory pointer to point to start of DRAM
	pruDRAM0_32int_ptr = (unsigned int*) memPointer;
	
	if(pruDRAM0_32int_ptr == NULL){
		printf("Pointer to DRAM failed to initialize!\n");
		return -1;
	}
	
	//zero out all PWM channels
	memset(pruDRAM0_32int_ptr, 0, PWM_CHANNELS*4);
	printf("zeroing out all PWM channels!\n");
	
	prussdrv_exec_program (PWM_PRU, "./pwm.bin");
	return 0;
}	

int disable_pru(){
	//first zero out all PWM channels
	memset(pruDRAM0_32int_ptr, 0, PWM_CHANNELS*4);
	printf("zeroing out all PWM channels!\n");

	//reset pwm  pointer to NULL so if init fails it doesn't point somewhere bad
	pruDRAM0_32int_ptr = NULL;
	
	//diable PRU
	prussdrv_pru_disable(PWM_PRU);
	prussdrv_exit ();
	
	printf("PRU disabled succesfully!\n");
	return 0;
}

int write_pwm_pulse_us(int channel, int us){
	
	//check if pru has been initialised properly
	if(pruDRAM0_32int_ptr == NULL){
		printf("ERROR: PRU PWM Controller not initialized\n");
		return -1;
	}
	

	//convert required pulse into number of PRU loops
	unsigned int num_of_loops = us*PULSE_TO_PRU_CYCLES;
	
	//write to PRU memory
	pruDRAM0_32int_ptr[channel - 1] = num_of_loops;
		

	return 0;
}

