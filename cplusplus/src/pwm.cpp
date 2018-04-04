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

	// load pwm period into PRU DRAM
	channels->pwm_period = PWM_PERIOD*PULSE_TO_PRU_CYCLES;

	//load PRU firmware
	prussdrv_exec_program(PWM_PRU, "./pwm.bin");

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
	
	//Capping Function:
	for(int i = 0; i<4; i++){
		if(this->channel_val[i]>2000)
			this->channel_val[i] = 2000;

		else if(this->channel_val[i]<1000)
			this->channel_val[i] = 1000;
	}

	//load latest PWM signals into PRU DRAM
	channels->ch1 = this->channel_val[0]*PULSE_TO_PRU_CYCLES;
	channels->ch2 = this->channel_val[1]*PULSE_TO_PRU_CYCLES;
	channels->ch3 = this->channel_val[2]*PULSE_TO_PRU_CYCLES;
	channels->ch4 = this->channel_val[3]*PULSE_TO_PRU_CYCLES;

	return 0;
}

void Motors::demux_torques_to_pwm(){

	float mean_thrust = 4.00f*(float(recv->recv_channel[2]) - THRUST_CONST)*0.367;
	float thrusts[4];
    
	thrusts[0] = (0.25)*(mean_thrust - (torques[2]) - (torques[0] + torques[1])*MOM_COEFF);
	thrusts[1] = (0.25)*(mean_thrust - (torques[2]) + (torques[0] + torques[1])*MOM_COEFF);
	thrusts[2] = (0.25)*(mean_thrust + (torques[2]) + (torques[0] - torques[1])*MOM_COEFF);
	thrusts[3] = (0.25)*(mean_thrust + (torques[2]) - (torques[0] - torques[1])*MOM_COEFF);
	
	//cout  << thrusts[0] << " | " << thrusts[1] << " | " << thrusts[2] << " | " << thrusts[3] << endl;

	channel_val[0] = thrusts[0]*THRUST_COEFF + THRUST_CONST;
	channel_val[1] = thrusts[1]*THRUST_COEFF + THRUST_CONST;
	channel_val[2] = thrusts[2]*THRUST_COEFF + THRUST_CONST;
	channel_val[3] = thrusts[3]*THRUST_COEFF + THRUST_CONST;
	
	//cout << channel_val[0] << " | " << channel_val[1] << " | " << channel_val[2] << " | " << channel_val[3] << endl; 


}

Motors::Motors(Receiver *recv_ptr){

	// make channel pointer point to struct
	this->channels = &this->p;

	// set all flags to false
	this->is_pru_initialized = false;
	this->is_armed = false;

	// link Receiver
	this->recv = recv_ptr;

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
	int i = 0;

	//New Arming Procedure : Channel 4 maximum--> Channel 3 minimum --> Channel 5 maximum
    if(recv->recv_channel[4] < 1950) cout << "Set Channel 5 to Maximum To begin Arming Routine!" << endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	recv->update();
	if(recv->recv_channel[4] > 1950){
	    cout << "Hold Throttle Down and Move Up Channel 6 to maximum" << endl;
	    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	    recv->update();
	    while(recv->recv_channel[2] < recv->cal_throttle[0]+30 && recv->recv_channel[5] > 1900){
	      i++ ;
	      if(i == 10){
	        this->is_armed = true;
	        break;
	      }
	      std::this_thread::sleep_for(std::chrono::milliseconds(200));
	      recv->update();
	    }
	}
	
	if (this->is_armed){
		cout << "Motors armed!\n";
		this->set_motors_spool_rate();
		return 0;
	 }
	else
		return -1;
}

void Motors::set_motors_spool_rate(){

	// set all motor channels to spool rate
	this->channel_val[0] = MOTOR_SPOOL_RATE;
	this->channel_val[1] = MOTOR_SPOOL_RATE;
	this->channel_val[2] = MOTOR_SPOOL_RATE;
	this->channel_val[3] = MOTOR_SPOOL_RATE;

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
