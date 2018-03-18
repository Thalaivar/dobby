#include "dobby.h"

int Dobby::pre_flight_checks(){

  // check current state
  if(this->state == READY_TO_FLY){
    cerr << "DOBBY IS READY!";
    return 0;
  }

  // check IMU
  if(!imu.is_initialized){
    cerr << "IMU not initialized!\n";
    return -1;
  }

  // check Receiver
  if(!radio.is_radio_initialized){
    cerr << "Receiver not initialized!\n";
    return -1;
  }

  // check if flight mode is set
  if(mode.current_mode == NOT_SET){
    cerr << "Flight mode not set, cant take off!\n";
    return -1;
  }

  // any checks for controller?

  // check Motors
  if(!motors.is_pru_initialized){
    std::cerr << "Motors are not initialized" << '\n';
    return -1;
  }

  // check if IMU is calibrated
  if(!imu.is_calibrated){
    std::cerr << "IMU not calibrated!" << '\n';
    return -1;
  }

  // check if radio calibrated
  if(!radio.is_calibrated){
    std::cerr << "Receiver not calibrated!" << '\n';
    return -1;
  }

  // if everything checks out, ready to fly!
  cerr << "DOBBY IS READY!\n";
  this->state = READY_TO_FLY;
  return 0;
}

int Dobby::setup(){

  // initialize the radio
  if(radio.init_radio() < 0){
    std::cerr << "Radio failed to initialize!" << '\n';
    return -1;
  }

  // initialize the Motors
  if(motors.initialize_pru() < 0){
    std::cerr << "Motors failed to initialize!" << '\n';
    return -1;
  }

  //initialize the IMU
  if(imu.init_imu() < 0){
    std::cerr << "IMU failed to initialize" << '\n';
    return -1;
  }
  
  imu.set_initialYaw();
  
  imu.zero_initial_attitude();
  
  return 0;
}

void Dobby::control_loop(){
  
  // get latest imu data
  imu.update();

  // get latest radio signals
  radio.update();
  
  // get desired stuff
  mode.flight_mode_update();

  // call smc controller
  //control.run_smc_controller();
  
  imu.print_tb_angles();

  // get pwm signals
  motors.demux_torques_to_pwm(); 
  
 // cout << imu.body_rates_rotated[ROLL] << " | " << imu.body_rates_rotated[PITCH] << " | " << imu.body_rates_rotated[YAW] << endl;
 // cout << radio.recv_channel[0] << " | " << radio.recv_channel[1] << " | " << radio.recv_channel[2] << " | " << radio.recv_channel[3] << endl;
 // cout << motors.channel_val[0] << " | " << motors.channel_val[1] << " | " \
       << motors.channel_val[2] << " | " << motors.channel_val[3] << endl;
}

Dobby::Dobby(){
}
