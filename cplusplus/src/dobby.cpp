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

  return 0;
}

void Dobby::control_loop(){

  // get latest radio signals
  radio.update();

  motors.channel_val[0] = radio.recv_channel[2];
  motors.channel_val[1] = radio.recv_channel[2];
  motors.channel_val[2] = radio.recv_channel[2];
  motors.channel_val[3] = radio.recv_channel[2];

  motors.update();

  mode.flight_mode_update();
  control.print_attitude_error();
}

Dobby::Dobby(){
}
