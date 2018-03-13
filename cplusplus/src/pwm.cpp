#include "pwm.h"

int Motors::initialize_pru(){

	if(this->is_pru_initialized){
		printf("PRU already initialised\n");
		return -1;
	}

	//reset channels pointer to NULL so it doesn't point somewhere bad
	channels = NULL;
	this->is_pru_initialized = false;

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

	// need to send low pulse on all pwm channels before loading
	channels->ch1 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch2 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch3 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch4 = ESC_LOW*PULSE_TO_PRU_CYCLES;

	//load PRU firmware
	prussdrv_exec_program (PWM_PRU, "./pwm.bin");

	this->is_pru_initialized = true;
	return 0;
}

int Motors::disable_pru(){

	//check if PRU already disabled
	if(!this->is_pru_initialized){
		printf("PRU already disabled!\n");
		return -1;
	}

	// need to send low pulse on all pwm channels before disabling
	channels->ch1 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch2 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch3 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch4 = ESC_LOW*PULSE_TO_PRU_CYCLES;

	//reset channels pointer to NULL so it doesn't point somewhere bad
	channels = NULL;

	//disable PRU
	prussdrv_pru_disable(PWM_PRU);
	prussdrv_exit ();

	this->is_pru_initialized = false;
	printf("PRU disabled succesfully!\n");
	return 0;
}

int Motors::update(){

	//check if pru has been initialised properly
	if(channels == NULL || !this->is_pru_initialized){
		printf("ERROR: PRU PWM Controller not initialized\n");
		return -1;
	}


	//load latest PWM signals into PRU DRAM
	channels->ch1 = this->channel_val[0]*PULSE_TO_PRU_CYCLES;
	channels->ch2 = this->channel_val[1]*PULSE_TO_PRU_CYCLES;
	channels->ch3 = this->channel_val[2]*PULSE_TO_PRU_CYCLES;
	channels->ch4 = this->channel_val[3]*PULSE_TO_PRU_CYCLES;

	return 0;
}

void Motors::demux_torques_to_pwm(){

	this->channel_val[0] = (1/4)*((recv->recv_channel[2]) - torques[2] - (torques[1] + torques[0]));
	this->channel_val[1] = (1/4)*((recv->recv_channel[2]) - torques[2] + (torques[1] + torques[0]));
	this->channel_val[2] = (1/4)*((recv->recv_channel[2]) + torques[2] + (torques[0] - torques[1]));
	this->channel_val[3] = (1/4)*((recv->recv_channel[2]) + torques[2] - (torques[0] - torques[1]));

}
Motors::Motors(Receiver *recv_ptr){

	// make channel pointer point to struct
	this->channels = &this->p;

	// set all flags to false
	this->is_pru_initialized = false;
	this->is_armed = false;

	// link Receiver
	this->recv = recv_ptr;

	// load pwm period into PRU DRAM
	channels->pwm_period = PWM_PERIOD*PULSE_TO_PRU_CYCLES;

}

int Motors::calibrate_esc(){
	if(!this->is_pru_initialized){
		printf("PRU not initialised!\n");
		return -1;
	}

	char response;

	//warn user that calibration is starting
	printf("ESC Calibration begun! Take all props off!\n");

	//begin sending low pulse
	printf("Sending 1000us pulse to all channels!\n");
	channels->ch1 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch2 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch3 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch4 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	printf("Connect ESCs and enter Y to send high pulse: ");
	scanf("%c", &response);

	if(response == 'Y'){
			printf("Sending 2000us pulse to all channels!\n");
			channels->ch1 = ESC_HIGH*PULSE_TO_PRU_CYCLES;
			channels->ch2 = ESC_HIGH*PULSE_TO_PRU_CYCLES;
			channels->ch3 = ESC_HIGH*PULSE_TO_PRU_CYCLES;
			channels->ch4 = ESC_HIGH*PULSE_TO_PRU_CYCLES;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	//set all channels to 0
	channels->ch1 = 0;
	channels->ch2 = 0;
	channels->ch3 = 0;
	channels->ch4 = 0;

	printf("ESCs calibrated successfully!\n");
	return 0;
}

int Motors::arm_motors(){

	// exit if motors already armed
	if(this->is_armed){
		cerr << "Motors already armed!\n";
		return 0;
	}
	cout << recv->recv_channel[0] << " | " << recv->recv_channel[1] << " | " << recv->recv_channel[2] << " | " << recv->recv_channel[3] << " | " << endl;
	// if user sends arm signal
	if(recv->recv_channel[2] < 1050 && recv->recv_channel[3] > 1990 && \
		 recv->recv_channel[1] < 1050 && recv->recv_channel[0] > 1990){
	
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	// check if arm signal was really arm signal
	if(recv->recv_channel[2] < 1050 && recv->recv_channel[3] > 1990 && \
		 recv->recv_channel[1] < 1050 && recv->recv_channel[0] > 1990){
			 this->is_armed = true;
			 cerr << "Motors armed!\n";
			 this->set_motors_spool_rate();
			 return 0;
		 }
	 }

	 // if it wasnt arm signal, or if nothing was sent, dont arm
	 return -1;
}

void Motors::set_motors_spool_rate(){

	// set all motor channels to spool rate
	this->channel_val[0] = MOTOR_SPOOL_RATE*PULSE_TO_PRU_CYCLES;
	this->channel_val[1] = MOTOR_SPOOL_RATE*PULSE_TO_PRU_CYCLES;
	this->channel_val[2] = MOTOR_SPOOL_RATE*PULSE_TO_PRU_CYCLES;
	this->channel_val[3] = MOTOR_SPOOL_RATE*PULSE_TO_PRU_CYCLES;

	// set motors to spin at spool rate
	this->update();
}

int Motors::disable_motors(){

	// cannot disable motors when they are armed
	if(this->is_armed){
		std::cerr << "Motors armed! cannot disable!" << '\n';
		return -1;
	}

	// send low pulse on all channels
	channels->ch1 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch2 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch3 = ESC_LOW*PULSE_TO_PRU_CYCLES;
	channels->ch4 = ESC_LOW*PULSE_TO_PRU_CYCLES;

	return -1;
}
