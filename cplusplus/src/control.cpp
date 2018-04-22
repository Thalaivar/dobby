#include "control.h"

Control::Control(Motors* motors_ptr, flightMode* flightMode_ptr, IMU* imu_ptr){

  // link imu, motors and flight_mode classes to controller
  this->motors = motors_ptr;
  this->mode = flightMode_ptr;
  this->imu = imu_ptr;

  // set all integral body rate errors to 0
  error.ie_body_rate[ROLL] = 0;
  error.ie_body_rate[PITCH] = 0;
  error.ie_body_rate[YAW] = 0;

}


flightMode::flightMode(Receiver *recv_ptr, IMU *imu_ptr){

  // link reciever class to get latest reciever data
  this->recv = recv_ptr;
  this->imu = imu_ptr;
}

void flightMode::flight_mode_update(){

  // check what current flight mode is
  switch(this->current_mode){

    case STABILIZE_ANGLE:
    // deadband for receiver
    if(recv->recv_channel[ROLL_CHANNEL] < 1510 && recv->recv_channel[ROLL_CHANNEL] > 1480){
      desired_euler[ROLL] = 0;
    }
    // get desired angle from pilot in degrees
    else{
      desired_euler[ROLL] = (float)((int)recv->recv_channel[ROLL_CHANNEL] - (recv->cal_roll[0] + recv->cal_roll[1])/2)*recv_signal_to_roll_angle;
     }

     // deadband for receiver
     if(recv->recv_channel[PITCH_CHANNEL] < 1510 && recv->recv_channel[PITCH_CHANNEL] > 1480){
       desired_euler[PITCH] = 0;
     }

     // get desired angle from pilot in degrees
     else{
       desired_euler[PITCH] = (float)((int)recv->recv_channel[PITCH_CHANNEL] - (recv->cal_pitch[0] + recv->cal_pitch[1])/2)*recv_signal_to_pitch_angle;
     }

     if(recv->recv_channel[YAW_CHANNEL] < 1510 && recv->recv_channel[YAW_CHANNEL] > 1480){
       desired_euler[YAW] = 0;
     }

      else{
        desired_euler[YAW] = (float)((int)recv->recv_channel[YAW_CHANNEL] - (recv->cal_yaw[0] + recv->cal_yaw[1])/2)*recv_signal_to_yaw_angle;
      }

	   break;

    default:
      cout << "FLIGHT MODE NOT SET!!\n";
      break;
  }
}

void Control::get_desired_euler_rates(){

  error.attitude_error[ROLL]  = (mode->desired_euler[ROLL] - imu->euler_angles[ROLL]*RAD_TO_DEG);
  error.attitude_error[PITCH] = (mode->desired_euler[PITCH] - imu->euler_angles[PITCH]*RAD_TO_DEG);

  // get desired angular rates (by passing through simple P controller)
  desired_euler_rates[ROLL] = error.attitude_error[ROLL]*angle_to_rate_roll;
  desired_euler_rates[PITCH] = error.attitude_error[PITCH]*angle_to_rate_pitch;
  desired_euler_rates[YAW] = mode->desired_euler[YAW]*angle_to_rate_yaw;

  desired_euler_rates[ROLL]  = desired_euler_rates[ROLL]*DEG_TO_RAD;
  desired_euler_rates[PITCH] = desired_euler_rates[PITCH]*DEG_TO_RAD;
  desired_euler_rates[YAW]   = desired_euler_rates[YAW]*DEG_TO_RAD;

}

void Control::get_desired_body_rates(){

  desired_body_rates[ROLL]  = desired_euler_rates[ROLL] - sin(imu->euler_angles[PITCH])*desired_euler_rates[YAW];
  desired_body_rates[PITCH] = desired_euler_rates[PITCH]*cos(imu->euler_angles[ROLL]) + desired_euler_rates[YAW]*sin(imu->euler_angles[ROLL])*cos(imu->euler_angles[PITCH]);
  desired_body_rates[YAW]   = cos(imu->euler_angles[ROLL])*cos(imu->euler_angles[PITCH])*desired_euler_rates[YAW] - sin(imu->euler_angles[ROLL])*desired_euler_rates[PITCH];
}

void Control::get_body_rate_error(){

  error.body_rate_error[ROLL]  = desired_body_rates[ROLL] - imu->body_rates[ROLL];
  error.body_rate_error[PITCH] = desired_body_rates[PITCH] - imu->body_rates[PITCH];
  error.body_rate_error[YAW]   = desired_body_rates[YAW] - imu->body_rates[PITCH];
}

void get_ie_body_rate_error(){
  // update the value of integral body rate error
  error.ie_body_rate[ROLL]  += error.body_rate_error[ROLL]*LOOP_TIME;
  error.ie_body_rate[PITCH] += error.body_rate_error[PITCH]*LOOP_TIME;
  error.ie_body_rate[YAW]   += error.body_rate_error[YAW]*LOOP_TIME;
}

void Control::run_smc_rate_controller(){

  // get latest desired euler rates
  get_desired_euler_rates();

  // get latest desired body rates
  get_desired_body_rates();

  // get latest body rate errors
  get_body_rate_error();

  // get latest integral errors
  get_ie_body_rate_error();

  // saturate the integral errors
  if(error.ie_body_rate[ROLL] > INTG_WNDP_ROLL_POS)
  	error.ie_body_rate[ROLL] = INTG_WNDP_ROLL_POS;
  else if(error.ie_body_rate[ROLL] < INTG_WNDP_ROLL_NEG)
  	error.ie_body_rate[ROLL] = INTG_WNDP_ROLL_NEG;
  if(error.ie_body_rate[PITCH] > INTG_WNDP_PITCH_POS)
  	error.ie_body_rate[PITCH] = INTG_WNDP_PITCH_POS;
  else if(error.ie_body_rate[PITCH] < INTG_WNDP_PITCH_NEG)
  	error.ie_body_rate[PITCH] = -INTG_WNDP_PITCH_NEG;

  // define the three sliding surfaces
  s_roll = error.body_rate_error[ROLL] + smc_roll_lambda*error.ie_body_rate[ROLL];
  s_pitch = error.body_rate_error[PITCH] + smc_pitch_lambda*error.ie_body_rate[PITCH];
  s_yaw = error.body_rate_error[YAW] + smc_yaw_lambda*error.ie_body_rate[YAW];

 // get controller outputs
  u_phi = (Izz - Iyy)*imu->body_rates[PITCH]*imu->body_rates[YAW] + Ixx*smc_roll_lambda*error.body_rate_error[ROLL] - smc_roll_eta*atan(s_roll);
  u_theta = (Ixx - Izz)*imu->body_rates[ROLL]*imu->body_rates[YAW] + Iyy*smc_pitch_lambda*error.body_rate_error[PITCH] - smc_pitch_eta*atan(s_pitch);
  u_psi = (Iyy - Ixx)*imu->body_rates[PITCH]*imu->body_rates[ROLL] + Izz*smc_yaw_lambda*error.body_rate_error[YAW] - smc_yaw_eta*atan(s_yaw);

  // update required torques
  motors->torques[ROLL] = u_phi;
  motors->torques[PITCH] = u_theta;
  motors->torques[YAW] = u_psi;
}


//CHECK AND CORRECT FOR ERRORS::
void Control::run_pid_controller(){

	// get latest desired euler rates
	get_desired_euler_rates();

	// get latest desired body rates
	get_desired_body_rates();

	// get latest body rate errors
	get_body_rate_error();

	// update the value of integral body rate error
	error.ie_body_rate[ROLL]  += error.body_rate_error[ROLL]*LOOP_TIME;
	error.ie_body_rate[PITCH] += error.body_rate_error[PITCH]*LOOP_TIME;
	error.ie_body_rate[YAW]   += error.body_rate_error[YAW]*LOOP_TIME;

	// Cap the Integral Error:
	if(error.ie_body_rate[ROLL] > INTG_WNDP_ROLL_POS)
		error.ie_body_rate[ROLL] = INTG_WNDP_ROLL_POS;
	else if(error.ie_body_rate[ROLL] < INTG_WNDP_ROLL_NEG)
		error.ie_body_rate[ROLL] = INTG_WNDP_ROLL_NEG;

	if(error.ie_body_rate[PITCH] > INTG_WNDP_PITCH_POS)
		error.ie_body_rate[PITCH] = INTG_WNDP_PITCH_POS;
	else if(error.ie_body_rate[PITCH] < INTG_WNDP_PITCH_NEG)
		error.ie_body_rate[PITCH] = -INTG_WNDP_PITCH_NEG;

	// update the value of derivative body rate error
	if(error.body_rate_error[ROLL] - error.Prevbody_rate_error[ROLL] != 0)
		error.body_rate_dError[ROLL]  = (error.body_rate_error[ROLL] - error.Prevbody_rate_error[ROLL])   * INV_LOOP_TIME;

	if(error.body_rate_error[PITCH] - error.Prevbody_rate_error[PITCH] != 0)
		error.body_rate_dError[PITCH] = (error.body_rate_error[PITCH] - error.Prevbody_rate_error[PITCH]) * INV_LOOP_TIME;

	if(error.body_rate_error[YAW] - error.Prevbody_rate_error[YAW] != 0)
		error.body_rate_dError[YAW]   = (error.body_rate_error[YAW] - error.Prevbody_rate_error[YAW])     * INV_LOOP_TIME;

	//Compute the Desired Torque Inputs:
	float u_phi = pid_roll_kp * error.body_rate_error[ROLL] + pid_roll_ki * error.ie_body_rate[ROLL] + pid_roll_kd * error.body_rate_dError[ROLL];
	float u_theta = pid_pitch_kp * error.body_rate_error[PITCH] + pid_pitch_ki * error.ie_body_rate[PITCH] + pid_pitch_kd * error.body_rate_dError[PITCH];
	float u_psi = pid_yaw_kp * error.body_rate_error[YAW] + pid_yaw_ki * error.ie_body_rate[YAW] + pid_yaw_kd * error.body_rate_dError[YAW];

	//SET VALUES FOR NEXT LOOP
	error.Prevbody_rate_error[ROLL] = error.body_rate_error[ROLL];
	error.Prevbody_rate_error[PITCH] = error.body_rate_error[PITCH];
	error.Prevbody_rate_error[YAW] = error.body_rate_error[YAW];

	// update required torques converting kg -> gm
	motors->torques[ROLL] = u_phi;
	motors->torques[PITCH] = u_theta;
	motors->torques[YAW] = u_psi;
}


void flightMode::set_flight_mode(flight_mode desired_mode){
  switch(desired_mode){

    case STABILIZE_ANGLE:
      this->current_mode = STABILIZE_ANGLE;
      break;

    case STABILIZE_RATES:
      this->current_mode = STABILIZE_RATES;
      break;

    default:
      cout << "INVALID MODE SETTING!\n";
      this->current_mode = NOT_SET;
      break;
  }
}

int sign(float x){
  if( x > 0.000 ) return 1;
  else if( x < 0.000 ) return -1;
  else return 0;
}
