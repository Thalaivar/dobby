#include "ppm.h"

int Receiver::initialize_pru(){
  if(this->is_pru_initialized){
		printf("PRU already initialised\n");
		return -1;
	}

  //reset channels pointer to NULL so it doesn't point somewhere bad
	ppm_channels = NULL;
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

  // start mmaping dram memory into channelPtr struct
	printf("mmaping PRU0 DRAM memory\n");
	prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, (void**) &ppm_channels);

  if(ppm_channels == NULL){
		printf("Pointer to DRAM failed to initialize!\n");
		return -1;
	}

  //load PRU firmware
	prussdrv_exec_program (PPM_PRU, "./ppm.bin");

	this->is_pru_initialized = true;
	return 0;
}

int Receiver::disable_pru(){

	//check if PRU already disabled
	if(!this->is_pru_initialized){
		printf("PRU already disabled!\n");
		return -1;
	}

	ppm_channels = NULL;

	//diable PRU
	prussdrv_pru_disable(PPM_PRU);
	prussdrv_exit ();

	this->is_pru_initialized = false;
	printf("PRU disabled succesfully!\n");
	return 0;
}

int Receiver::update(){
  //check if pru has been initialised properly
	if(ppm_channels == NULL || !this->is_pru_initialized){
		printf("ERROR: PRU PPM decoder not initialized\n");
		return -1;
	}
  //CAPPING PPM SIGNAL 1000-2000(Uncomment if required)
  /*
  for(int i=0; i<6; i++){
    if(this->recv_channel[i] > 2000)
      this->recv_channel[i] = 2000;
    else if(this->recv_channel[i] < 1000)
      this->recv_channel[i] = 1000;
  }
  */

  //load latest PPM values from PRU0 DRAM
	this->recv_channel[0] = ppm_channels->ch1*PRU_CYCLES_TO_US;
	this->recv_channel[1] = ppm_channels->ch2*PRU_CYCLES_TO_US;
	this->recv_channel[2] = ppm_channels->ch3*PRU_CYCLES_TO_US;
	this->recv_channel[3] = ppm_channels->ch4*PRU_CYCLES_TO_US;
	this->recv_channel[4] = ppm_channels->ch5*PRU_CYCLES_TO_US;
  this->recv_channel[5] = ppm_channels->ch6*PRU_CYCLES_TO_US;

  return 0;
}

Receiver::Receiver(void){

	// make channel pointer point to struct
	this->ppm_channels = &this->p;

	this->is_pru_initialized = false;
    this->is_radio_initialized = false;
    this->is_calibrated = false;


}

int Receiver::init_radio(){
  // initialize the PRU for PWM
	if(this->initialize_pru() < 0){
    cout << "Could not initialize PRU for PPM!!\n";
    return -1;
  }

  // check if radio is calibrated
  if(this->load_radio_cal() < 0){
    cout << "Radio not calibrated!\nRunning radio cal now... \n";
    if(this->calibrate_radio() < 0) return -1; }

  if(this->is_pru_initialized && this->is_calibrated){
    this->is_radio_initialized = true;
    return 0;
  }
}

int Receiver::load_radio_cal(){
  // set all calibration vals to 0, will be loaded if calibration has been done
  this->cal_roll[0] = 0;
  this->cal_pitch[0] = 0;
  this->cal_yaw[0] = 0;
  this->cal_throttle[0] = 0;
  this->cal_roll[1] = 0;
  this->cal_pitch[1] = 0;
  this->cal_yaw[1] = 0;
  this->cal_throttle[1] = 0;

  // load calibration values
  ifstream saveFile;
  saveFile.open("radio_cal.txt");

  // check if file exists
  if(saveFile.fail()) {
    cout << "Error opening radio cal file!" << endl;
    return -1;
  }

  saveFile >> this->cal_roll[0] >> this->cal_roll[1];
  saveFile >> this->cal_pitch[0] >> this->cal_pitch[1];
  saveFile >> this->cal_yaw[0] >> this->cal_yaw[1];
  saveFile >> this->cal_throttle[0] >> this->cal_throttle[1];

  this->is_calibrated = true;

  saveFile.close();
  return 0;
}

int Receiver::save_radio_cal(){

  // save radio valibration values
  ofstream saveFile;
  saveFile.open("radio_cal.txt");

  if(saveFile.fail()) {
    cerr << "Error opening file!" << endl;
    return -1;
  }

  saveFile << this->cal_roll[0] << '\n';
  saveFile << this->cal_roll[1] << '\n';
  saveFile << this->cal_pitch[0] << '\n';
  saveFile << this->cal_pitch[1] << '\n';
  saveFile << this->cal_yaw[0] << '\n';
  saveFile << this->cal_yaw[1] << '\n';
  saveFile << this->cal_throttle[0] << '\n';
  saveFile << this->cal_throttle[1] << '\n';

  return 0;
}

int Receiver::calibrate_radio(){

  this->is_radio_initialized = true;

  int temp_low = 0;
  int temp_high = 0;

  // prompt user
  cout << "Beginning radio calibration now....\n* Center all sticks\n* Keep throttle at minimum\n";
  cout << "* Keep all extra channels at minimum\n Enter \"y\" to continue: ";

  char choice;
  cin >> choice;

  if(choice == 'y'){
    int i = 0;

    // make sure user has kept all sticks correctly
    this->update();

    while(this->recv_channel[0] >= 1510 || this->recv_channel[0] <= 1490){
      if(i == 0) printf("Center roll stick!\n");
	  this->update();
	  i++;
    }
    i = 0;

    while(this->recv_channel[1] >= 1510 || this->recv_channel[1] <= 1490){
      if(i == 0) printf("Center pitch stick!\n");
      this->update();
	  i++;
    }
    i = 0;

    while(this->recv_channel[3] >= 1510 || this->recv_channel[3] <= 1490){
      if(i == 0) printf("Center yaw stick!\n");
       this->update();
	   i++;
    }
    i = 0;


    while(this->recv_channel[2] >= 1050){
      if(i == 0) printf("Keep throttle at 0!\n");
       this->update();
	   i++;
    }
    i = 0;

    while(this->recv_channel[4] >= 1020 || this->recv_channel[5] >= 1020){
      if(i == 0) printf("Keep all extra channels at 0!\n");
       this->update();
	   i++;
    }
    i = 0;

    /************************************************************************
                      start calibrating radio signals
    ************************************************************************/

    // calibrate roll channel
    cout << "Keep roll at minimum\nEnter \"y\" to continue: ";
    cin >> choice;

    if(choice == 'y'){
        for(int i = 0; i < 1000; i++){
           this->update();
           temp_low = temp_low + this->recv_channel[ROLL_CHANNEL];
        }

        temp_low = temp_low/1000;
    }

    cout << "Keep roll at maximum\nEnter \"y\" to continue: ";
    cin >> choice;

    if(choice == 'y'){
        for(int i = 0; i < 1000; i++){
           this->update();
           temp_high = temp_high + this->recv_channel[ROLL_CHANNEL];
        }

        temp_high = temp_high/1000;
    }

    this->cal_roll[0] = temp_low;
    this->cal_roll[1] = temp_high;

    // calibrate pitch channel
    cout << "Keep pitch at minimum\nEnter \"y\" to continue: ";
    cin >> choice;

    if(choice == 'y'){
        for(int i = 0; i < 1000; i++){
           this->update();
           temp_low = temp_low + this->recv_channel[PITCH_CHANNEL];
        }

        temp_low = temp_low/1000;
    }

    cout << "Keep pitch at maximum\nEnter \"y\" to continue: ";
    cin >> choice;

    if(choice == 'y'){
        for(int i = 0; i < 1000; i++){
           this->update();
           temp_high = temp_high + this->recv_channel[PITCH_CHANNEL];
        }

        temp_high = temp_high/1000;
    }

    this->cal_pitch[0] = temp_low;
    this->cal_pitch[1] = temp_high;

    // calibrate yaw channel
    cout << "Keep yaw at minimum\nEnter \"y\" to continue: ";
    cin >> choice;

    if(choice == 'y'){
        for(int i = 0; i < 1000; i++){
           this->update();
           temp_low = temp_low + this->recv_channel[YAW_CHANNEL];
        }

        temp_low = temp_low/1000;
    }

    cout << "Keep yaw at maximum\nEnter \"y\" to continue: ";
    cin >> choice;

    if(choice == 'y'){
        for(int i = 0; i < 1000; i++){
           this->update();
           temp_high = temp_high + this->recv_channel[YAW_CHANNEL];
        }

        temp_high = temp_high/1000;
    }

    this->cal_yaw[0] = temp_low;
    this->cal_yaw[1] = temp_high;

    // calibrate throttle channel
    cout << "Keep throttle at minimum\nEnter \"y\" to continue: ";
    cin >> choice;

    if(choice == 'y'){
        for(int i = 0; i < 1000; i++){
           this->update();
           temp_low = temp_low + this->recv_channel[THROTTLE_CHANNEL];
        }

        temp_low = temp_low/1000;
    }

    cout << "Keep throttle at maximum\nEnter \"y\" to continue: ";
    cin >> choice;

    if(choice == 'y'){
        for(int i = 0; i < 1000; i++){
           this->update();
           temp_high = temp_high + this->recv_channel[THROTTLE_CHANNEL];
        }

        temp_high = temp_high/1000;
    }

    this->cal_throttle[0] = temp_low;
    this->cal_throttle[1] = temp_high;


    if(this->save_radio_cal() < 0) return -1;
    this->is_calibrated = true;

    this->is_radio_initialized = false;
    return 0;
  }

  else{
    cout << "Calibration not done!\n";
    return -1;
  }
}
