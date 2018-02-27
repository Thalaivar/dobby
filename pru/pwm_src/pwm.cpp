#include "pwm.h"

int Motors::initialize_pru(){

	if(this->is_initialized){
		printf("PRU already initialised\n");
		return -1;
	}

	//reset channels pointer to NULL so it doesn't point somewhere bad
	channels = NULL;
	this->is_initialized = false;

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

	//// start mmaping dram memory into channelPtr struct
	printf("mmaping PRU1 DRAM memory\n");
	static void* memPointer;
	prussdrv_map_prumem(PRUSS0_PRU1_DATARAM, (void**) &channels);


	if(channels == NULL){
		printf("Pointer to DRAM failed to initialize!\n");
		return -1;
	}

	//zero out all PWM channels
	//memset(channels, 0, PWM_CHANNELS*4);
	//printf("zeroing out all PWM channels!\n");

	//load PRU firmware
	prussdrv_exec_program (PWM_PRU, "./pwm.bin");

	this->is_initialized = true;
	return 0;
}

int Motors::disable_pru(){

	//check if PRU already disabled
	if(!this->is_initialized){
		printf("PRU already disabled!\n");
		return -1;
	}

	//first zero out all PWM channels
	//memset(channels, 0, PWM_CHANNELS*4);
	//printf("zeroing out all PWM channels!\n");

	//reset channels pointer to NULL so it doesn't point somewhere bad
	channels = NULL;

	//diable PRU
	prussdrv_pru_disable(PWM_PRU);
	prussdrv_exit ();

	this->is_initialized = false;
	printf("PRU disabled succesfully!\n");
	return 0;
}

int Motors::update(){

	//check if pru has been initialised properly
	if(channels == NULL || !this->is_initialized){
		printf("ERROR: PRU PWM Controller not initialized\n");
		return -1;
	}

	//load latest PWM signals into PRU DRAM
	channels->ch1 = this->channel_val[0];
	channels->ch2 = this->channel_val[1];
	channels->ch3 = this->channel_val[2];
	channels->ch4 = this->channel_val[3];

	return 0;
}

Motors::Motors(void){

	// make channel pointer point to struct
	this->channels = &this->p;
	this->is_initialized = false;

	// initialize the PRU for PWM
	if(this->initialize_pru() < 0) printf("Could not initialize PRU for PWM!!\n");

	// load pwm period into PRU DRAM
	channels->pwm_period = PWM_PERIOD;

}

int Motors::calibrate_esc(){
	if(!this->is_initialized){
		printf("PRU not initialised!\n");
		return -1;
	}

	char response;

	//warn user that calibration is starting
	printf("ESC Calibration begun! Take all props off!\n");

	//begin sending low pulse
	printf("Sending 1000us pulse to all channels!\n");
	channels->ch1 = ESC_LOW;
	channels->ch2 = ESC_LOW;
	channels->ch3 = ESC_LOW;
	channels->ch4 = ESC_LOW;
	printf("Connect ESCs and enter Y to send high pulse: ");
	scanf("%c", &response);

	if(response == 'Y'){
			printf("Sending 2000us pulse to all channels!\n");
			channels->ch1 = ESC_HIGH;
			channels->ch2 = ESC_HIGH;
			channels->ch3 = ESC_HIGH;
			channels->ch4 = ESC_HIGH;
	}

	//set all channels to 0
	channels->ch1 = 0;
	channels->ch2 = 0;
	channels->ch3 = 0;
	channels->ch4 = 0;

	printf("ESCs calibrated successfully!\n");
	return 0;
}
