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

  if(rc_set_cpu_freq(FREQ_1000MHZ) < 0){
  	std::cerr << "CPU frequency setting failed!" << '\n';
	return -1;
  }

  imu.set_initialYaw();


  return 0;
}

void Dobby::control_loop(dobby_time current_time){

  auto fast_loop = chrono::duration_cast<chrono::microseconds>(current_time - times.fast_loop_prev_time);
  times.fast_loop_time = fast_loop.count();


  if(times.fast_loop_time < FASTLOOP_PERIOD)
    return;

  else{

    times.fast_loop_prev_time = current_time;

    // get latest attitude
    imu.update();

    // get desired stuff
    mode.flight_mode_update();

    // call smc controller
    control.run_smc_controller();

    return;
  }
}

void Dobby::radio_update_loop(dobby_time current_time){

  auto radio_loop = chrono::duration_cast<chrono::microseconds>(current_time - times.radio_loop_prev_time);
  times.radio_loop_time = radio_loop.count();

  if(times.radio_loop_time < RADIO_LOOP_PERIOD)
      return;

  else{
	times.radio_loop_prev_time = current_time;

    // get latest radio signals
    radio.update();

    return;
  }
}

void Dobby::motor_update_loop(dobby_time current_time){

  auto motor_loop = chrono::duration_cast<chrono::microseconds>(current_time - times.motor_loop_prev_time);
  times.motor_loop_time = motor_loop.count();

  if(times.motor_loop_time < MOTOR_LOOP_PERIOD)
      return;

  else{

    times.motor_loop_prev_time = current_time;

    // get pwm signals
    motors.demux_torques_to_pwm();

    motors.update();
    // cout << imu.body_rates[ROLL] << " | " << imu.body_rates[PITCH] << " | " << imu.body_rates[YAW] << endl;
    // cout << radio.recv_channel[0] << " | " << radio.recv_channel[1] << " | " << radio.recv_channel[2] << " | " << radio.recv_channel[3] << endl;
    // cout << motors.torques[0] << " | " << motors.torques[1] << " | " << motors.torques[2] << endl;

    return;
  }
}

void Dobby::logging_loop(dobby_time current_time){

	auto log_loop = chrono::duration_cast<chrono::microseconds>(current_time - times.logging_loop_prev_time);
  	times.logging_loop_time = log_loop.count();

  	if(times.logging_loop_time < LOG_LOOP_PERIOD)
      return;

	else{
		times.logging_loop_prev_time = current_time;
		logging.log_s(control.s_roll, control.s_pitch, control.s_yaw);
    logging.log_attitude_error(control.error.angle_error[ROLL], control.error.angle_error[PITCH], control.error.angle_error[YAW]);
    logging.log_ie_body_rate_error(control.error.ie_body_rate[ROLL], control.error.ie_body_rate[PITCH], control.error.ie_body_rate[YAW]);
	}
}

void Dobby::reset_all_times(){

  dobby_time t1 = timer::now();
  times.fast_loop_prev_time = t1;
  times.radio_loop_prev_time = t1;
  times.motor_loop_prev_time = t1;

}
