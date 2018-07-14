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
  cout << "DOBBY IS READY!\n";
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

  if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)< 0){
  	std::cerr << "CPU frequency setting failed!" << '\n';
	return -1;
  }

  return 0;
}

int Dobby::one_dof_setup(){
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

  // max CPU performance
  if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)< 0){
  	std::cerr << "CPU frequency setting failed!" << '\n';
	return -1;
  }

  // set flight_mode for 1DOF testing
  mode.set_flight_mode(ONE_DOF_TEST);

  // check IMU
  if(!imu.is_initialized){
    std::cerr << "IMU not initialized!\n";
    return -1;
  }

  // check if flight mode is set
  if(mode.current_mode == NOT_SET){
    std::cerr << "Flight mode not set, cant take off!\n";
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

  cout << "************************\n*  Ready for 1DOF test *\n************************" << endl;
  this->state = ONE_DOF_TEST_READY;
  return 0;
}

int Dobby::pwm_test_setup(){
  // initialize the Motors
  if(motors.initialize_pru() < 0){
    std::cerr << "Motors failed to initialize!" << '\n';
    return -1;
  }

  if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)< 0){
    std::cerr << "CPU frequency setting failed!" << '\n';
  return -1;
  }

  return 0;
}

int Dobby::imu_test_setup(){
  //initialize the IMU
  if(imu.init_imu() < 0){
    std::cerr << "IMU failed to initialize" << '\n';
    return -1;
  }

  if(rc_cpu_set_governor(RC_GOV_PERFORMANCE)< 0){
  	std::cerr << "CPU frequency setting failed!" << '\n';
	return -1;
  }

  return 0;
}

void Dobby::imu_test_update_loop(dobby_time current_time){
    auto imu_test_loop = chrono::duration_cast<chrono::microseconds>(current_time - times.imu_test_update_loop_prev_time);
    times.imu_test_update_loop_time = imu_test_loop.count();

    if(times.imu_test_update_loop_time < FASTLOOP_PERIOD)
      return;

    else{

      times.imu_test_update_loop_prev_time = current_time;

      imu.update();
      return;

    }
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

void Dobby::control_loop_1DOF(dobby_time current_time){
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
    control.run_pid_controller();

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

void Dobby::motor_update_loop_1DOF(dobby_time current_time){
  auto motor_loop = chrono::duration_cast<chrono::microseconds>(current_time - times.motor_loop_prev_time);
  times.motor_loop_time = motor_loop.count();

  if(times.motor_loop_time < MOTOR_LOOP_PERIOD)
      return;

  else{

    times.motor_loop_prev_time = current_time;

    // get pwm signals
    motors.demux_torques_to_pwm_1DOF();

    motors.update();
    // cout << imu.body_rates[ROLL] << " | " << imu.body_rates[PITCH] << " | " << imu.body_rates[YAW] << endl;
    // cout << radio.recv_channel[0] << " | " << radio.recv_channel[1] << " | " << radio.recv_channel[2] << " | " << radio.recv_channel[3] << endl;
    // cout << motors.torques[0] << " | " << motors.torques[1] << " | " << motors.torques[2] << endl;

    return;
  }
}
void Dobby::pwm_test_loop(dobby_time current_time, int* desired_test_pwm){
  auto motor_loop = chrono::duration_cast<chrono::microseconds>(current_time - times.motor_loop_prev_time);
  times.motor_loop_time = motor_loop.count();

  if(times.motor_loop_time < MOTOR_LOOP_PERIOD)
    return;

  else{
	int test_pwm[4] = {*desired_test_pwm, *(desired_test_pwm++), *(desired_test_pwm++), *(desired_test_pwm++)};
    times.motor_loop_prev_time = current_time;
    motors.channel_val[0] = test_pwm[0];
    motors.channel_val[1] = test_pwm[1];
    motors.channel_val[2] = test_pwm[2];
    motors.channel_val[3] = test_pwm[3];
    motors.update();
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
		//logging.log_s(control.s_roll, control.s_pitch, control.s_yaw);
    	logging.log_channel_vals(motors.channel_val[0], motors.channel_val[1], motors.channel_val[2], motors.channel_val[3]);
		logging.log_ie_body_rate_error(control.error.ie_body_rate[ROLL], control.error.ie_body_rate[PITCH], control.error.ie_body_rate[YAW]);
		logging.log_attitude_error(imu.euler_angles[ROLL], imu.euler_angles[PITCH], imu.euler_angles[YAW]);
    	logging.log_body_rate_error(control.error.body_rate_error[ROLL], control.error.body_rate_error[PITCH], control.error.body_rate_error[YAW]);    
		logging.log_desired_body_rates(control.desired_body_rates[ROLL], control.desired_body_rates[PITCH], control.desired_body_rates[YAW]);
	}
}

void Dobby::imu_test_logging_loop(dobby_time current_time){
  auto imu_test_log_loop = chrono::duration_cast<chrono::microseconds>(current_time - times.imu_test_log_loop_prev_time);
  	times.imu_test_log_loop_time = imu_test_log_loop.count();

    if(times.imu_test_log_loop_time < LOG_LOOP_PERIOD)
      return;

    else{
      times.imu_test_log_loop_prev_time = current_time;
      logging.log_attitude(imu.euler_angles[ROLL], imu.euler_angles[PITCH], imu.euler_angles[YAW]);
      return;
    }
}
void Dobby::reset_all_times(){

  dobby_time t1 = timer::now();
  times.fast_loop_prev_time = t1;
  times.radio_loop_prev_time = t1;
  times.motor_loop_prev_time = t1;

}
