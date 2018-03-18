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

  prev_time = 0;

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


   // get desired angular rates (by passing through simple P controller)
      desired_euler_rates[ROLL] = (imu->euler_angles[ROLL]*RAD_TO_DEG - desired_euler[ROLL])*angle_to_rate_roll;
      desired_euler_rates[PITCH] = (imu->euler_angles[PITCH]*RAD_TO_DEG - desired_euler[PITCH])*angle_to_rate_pitch;
      desired_euler_rates[YAW] = (imu->euler_angles[YAW]*RAD_TO_DEG - desired_euler[YAW])*angle_to_rate_yaw;

	 break;

    default:
      cout << "FLIGHT MODE NOT SET!!\n";
      break;
  }
}

void Control::get_desired_body_rates(){

  // variables to make euler rates -> body rates transformation more readable
  float phi = imu->euler_angles[ROLL];
  float theta = imu->euler_angles[PITCH];
  float psi = imu->euler_angles[YAW];
  float phi_dot_des = mode->desired_euler_rates[ROLL];
  float theta_dot_des = mode->desired_euler_rates[PITCH];
  float psi_dot_des = mode->desired_euler_rates[YAW];

  desired_body_rates[ROLL]  = cos(theta)*cos(psi)*phi_dot_des + sin(psi)*theta_dot_des;
  desired_body_rates[PITCH] = -cos(theta)*sin(psi)*phi_dot_des + cos(psi)*theta_dot_des;
  desired_body_rates[YAW]   = sin(theta)*phi_dot_des + psi_dot_des;

 // cout << desired_body_rates[ROLL] << " | " << desired_body_rates[PITCH] << " | " << desired_body_rates[YAW] << endl;
}

void Control::get_body_rate_error(){

  // <angle>_rate_error = current_<angle>_rate - desired_<angle>_rate
  error.body_rate_error[ROLL]  = imu->body_rates_rotated[ROLL] - desired_body_rates[ROLL];
  error.body_rate_error[PITCH] = imu->body_rates_rotated[PITCH] - desired_body_rates[PITCH];
  error.body_rate_error[YAW]   = imu->body_rates_rotated[YAW] - desired_body_rates[YAW];
}

void Control::run_smc_controller(){

  // get loop time (will be a fixed time later on)
  t = clock();
  float  dt = (float)(t - prev_time)/CLOCKS_PER_SEC;
  prev_time = t;

  // declare state variables
  float wx = imu->body_rates[ROLL];
  float wy = imu->body_rates[PITCH];
  float wz = imu->body_rates[YAW];

  // get latest desired body rates
  get_desired_body_rates();

  // get latest body rate errors
  get_body_rate_error();

  //cout << error.body_rate_error[0] << " | " << error.body_rate_error[1] << " | " << error.body_rate_error[2] << endl;

  // define the three sliding surfaces
  float s_phi = Ixx*error.body_rate_error[ROLL] + smc_roll_lambda*error.ie_body_rate[ROLL];
  float s_theta = Iyy*error.body_rate_error[PITCH] + smc_pitch_lambda*error.ie_body_rate[PITCH];
  float s_psi = Izz*error.body_rate_error[YAW] + smc_yaw_lambda*error.ie_body_rate[YAW];

  //cout << s_phi << " | " << s_theta << " | " << s_psi << endl;
  // get controller outputs
  float u_phi = (Izz - Iyy)*wy*wz - smc_roll_lambda*error.body_rate_error[ROLL] - smc_roll_eta*sign(s_phi);
  float u_theta = (Ixx - Izz)*wx*wz - smc_pitch_lambda*error.body_rate_error[PITCH] - smc_pitch_eta*sign(s_theta);
  float u_psi = (Iyy - Ixx)*wy*wx - smc_yaw_lambda*error.body_rate_error[YAW] - smc_yaw_eta*sign(s_psi);

 // cout << u_phi << " | " << u_theta << " | " << u_psi << endl;

  // update the value of integral body rate error
  error.ie_body_rate[ROLL]  += error.body_rate_error[ROLL]*dt;
  error.ie_body_rate[PITCH] += error.body_rate_error[PITCH]*dt;
  error.ie_body_rate[YAW]   += error.body_rate_error[YAW]*dt;

  // update required torques
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

void Control::demux_control_signal(){

  // Τ_x = I_x*(u_theta*sin(psi) + u_phi*cos(psi))
  motors->torques[ROLL] = Ixx*(control_signal[PITCH]*sin(imu->euler_angles[YAW]) \
                             + control_signal[ROLL]*cos(imu->euler_angles[YAW]));

  // Τ_y = I_y*(u_theta*cos(psi) - u_phi*sin(psi))
  motors->torques[PITCH] = Iyy*(control_signal[PITCH]*cos(imu->euler_angles[YAW]) \
                              - control_signal[ROLL]*sin(imu->euler_angles[YAW]));

  // T_z = I_z*(u_psi + tan(theta)*u_phi)
  motors->torques[YAW] = Izz*(control_signal[YAW] + \
                            tan(imu->euler_angles[PITCH])*control_signal[ROLL]);
}

int sign(float x){
  if( x > 0.000 ) return 1;
  else if( x < 0.000 ) return -1;
  else return 0;
}

void Control::print_body_rate_error(){

  get_body_rate_error();

  cout << error.body_rate_error[ROLL] << " | " << error.body_rate_error[PITCH] << " | " << error.body_rate_error[YAW] << endl;
}

void flightMode::print_desired_attitude(){

  cout << this->desired_euler[0] << " | " << this->desired_euler[1] << " | " << this->desired_euler[2] << endl;
}

void flightMode::print_desired_attitude_rates(){

  cout << this->desired_euler_rates[0] << " | " << this->desired_euler_rates[1] << " | " << this->desired_euler_rates[2] << endl;
}
