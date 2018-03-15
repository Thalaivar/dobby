#include "control.h"

Control::Control(Motors* motors_ptr, flightMode* flightMode_ptr){

  // link and motors and flight_mode classes to controller
  this->motors = motors_ptr;
  this->mode = flightMode_ptr;
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
      this->desired_attitude[ROLL] = 0;
    }
    // get desired angle from pilot in degrees
    else{
      this->desired_attitude[ROLL] = (float)((int)recv->recv_channel[ROLL_CHANNEL] - (recv->cal_roll[0] + recv->cal_roll[1])/2)*recv_signal_to_roll_angle;
     }

     // deadband for receiver
     if(recv->recv_channel[PITCH_CHANNEL] < 1510 && recv->recv_channel[PITCH_CHANNEL] > 1480){
       this->desired_attitude[PITCH] = 0;
     }

     // get desired angle from pilot in degrees
     else{
         this->desired_attitude[PITCH] = (float)((int)recv->recv_channel[PITCH_CHANNEL] - (recv->cal_pitch[0] + recv->cal_pitch[1])/2)*recv_signal_to_pitch_angle;
     }

     if(recv->recv_channel[YAW_CHANNEL] < 1510 && recv->recv_channel[YAW_CHANNEL] > 1480){
       this->desired_attitude[YAW] = 0;
     }

      else{
        this->desired_attitude[YAW] = (float)((int)recv->recv_channel[YAW_CHANNEL] - (recv->cal_yaw[0] + recv->cal_yaw[1])/2)*recv_signal_to_yaw_angle;
      }

    // get desired angular rates (by passing through simple P controller)
      this->desired_attitude_rates[ROLL] = (imu->data.fused_TaitBryan[IMU_ROLL]*RAD_TO_DEG - this->desired_attitude[ROLL])*angle_to_rate_roll;
      this->desired_attitude_rates[PITCH] = (imu->data.fused_TaitBryan[IMU_PITCH]*RAD_TO_DEG - this->desired_attitude[PITCH])*angle_to_rate_pitch;
      this->desired_attitude_rates[YAW] = (imu->data.fused_TaitBryan[IMU_YAW]*RAD_TO_DEG - this->desired_attitude[YAW])*angle_to_rate_yaw;
      break;

    default:
      cout << "FLIGHT MODE NOT SET!!\n";
      break;
  }
}

void Control::get_attitude_error(){
  // <angle>_error = current_<angle> - desired_<angle>
  error.attitude_error[ROLL] = mode->imu->data.fused_TaitBryan[IMU_ROLL]*RAD_TO_DEG - mode->desired_attitude[ROLL];
  error.attitude_error[PITCH] = mode->imu->data.fused_TaitBryan[IMU_PITCH]*RAD_TO_DEG - mode->desired_attitude[PITCH];
  error.attitude_error[YAW] = mode->imu->data.fused_TaitBryan[IMU_YAW]*RAD_TO_DEG - mode->desired_attitude[YAW];
}

void Control::get_attitude_rate_error(){
  // <angle>_rate_error = current_<angle>_rate - desired_<angle>_rate
  error.attitude_rate_error[ROLL] = mode->imu->data.gyro[IMU_ROLL] - mode->desired_attitude_rates[ROLL];
  error.attitude_rate_error[PITCH] = mode->imu->data.gyro[IMU_PITCH] - mode->desired_attitude_rates[PITCH];
  error.attitude_rate_error[YAW] = mode->imu->data.gyro[IMU_YAW] - mode->desired_attitude_rates[YAW];
}

void Control::run_smc_controller(){

  // to make algebra simpler, use different variables to represent parts of dynamics
  float f1, f2, f3, g1, g2;

  // declare state variables
  float phi, theta, psi, phi_dot, theta_dot, psi_dot;

  // get euler euler_rates
  this->body_to_euler_rates();
  
  // assign state variables
  phi = mode->imu->data.fused_TaitBryan[IMU_ROLL];
  theta = mode->imu->data.fused_TaitBryan[IMU_PITCH];
  psi = mode->imu->data.fused_TaitBryan[IMU_YAW];
  phi_dot = this->euler_rates[ROLL];
  theta_dot = this->euler_rates[PITCH];
  psi_dot = this->euler_rates[YAW];
 
  // assign representative variables
  f1 = theta_dot*psi_dot*cos(psi) - theta_dot*phi_dot*sin(theta)*cos(psi) \
           - psi_dot*phi_dot*cos(theta)*sin(psi);
  f2 = phi_dot*theta_dot*sin(theta)*sin(psi) - phi_dot*psi_dot*cos(theta)*cos(psi) \
           - theta_dot*psi_dot*sin(psi);
  f3 = phi_dot*theta_dot*cos(theta);
  g1 = ((Izz - Iyy)/Ixx)*mode->imu->data.gyro[IMU_PITCH]*mode->imu->data.gyro[IMU_YAW];
  g2 = ((Ixx - Izz)/Iyy)*mode->imu->data.gyro[IMU_ROLL]*mode->imu->data.gyro[IMU_YAW];

  // get errors
  get_attitude_error();
  get_attitude_rate_error();

  // define sliding surfaces
  float s_roll = error.attitude_rate_error[ROLL] + smc_roll_lambda*error.attitude_error[ROLL];
  float s_pitch = error.attitude_rate_error[PITCH] + smc_pitch_lambda*error.attitude_error[PITCH];
  float s_yaw = error.attitude_rate_error[YAW] + smc_yaw_lambda*error.attitude_error[YAW];

  // run standard smc if mode is simple stabilise mode with pilot inputs as angles
  if(mode->current_mode == STABILIZE_ANGLE){
    control_signal[ROLL] = -f2*sin(psi) + f1*cos(psi) - g2*sin(psi) + g1*cos(psi) + \
                          cos(theta)*(-smc_roll_lambda*error.attitude_rate_error[ROLL]) - smc_roll_eta*sign(s_roll);
    control_signal[PITCH] = f2*cos(psi) + f1*sin(psi) + g2*cos(psi) + g1*sin(psi) - smc_pitch_lambda*error.attitude_rate_error[PITCH] \
                              - smc_pitch_eta*sign(s_pitch);
    control_signal[YAW] = tan(theta)*((f2 + g2)*sin(psi) - (f1 + g1)*cos(psi)) + f3 - smc_yaw_lambda*error.attitude_rate_error[YAW] \
                              - smc_yaw_eta*sign(s_yaw);
  }

  // get required torques
  demux_control_signal();
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
  motors->torques[ROLL] = Ixx*(control_signal[PITCH]*sin(mode->imu->data.fused_TaitBryan[TB_YAW_Z]) \
                             + control_signal[ROLL]*cos(mode->imu->data.fused_TaitBryan[TB_YAW_Z]));

  // Τ_y = I_y*(u_theta*cos(psi) - u_phi*sin(psi))
  motors->torques[PITCH] = Iyy*(control_signal[PITCH]*cos(mode->imu->data.fused_TaitBryan[TB_YAW_Z]) \
                              - control_signal[ROLL]*sin(mode->imu->data.fused_TaitBryan[TB_YAW_Z]));

  // T_z = I_z*(u_psi + tan(theta)*u_phi)
  motors->torques[YAW] = Izz*(control_signal[YAW] + \
                            tan(mode->imu->data.fused_TaitBryan[TB_PITCH_X])*control_signal[ROLL]);
}

void Control::body_to_euler_rates(){
  // state variables
  float theta, phi, psi;

  // get attitude from imu, it is in terms of euler angles
  phi = mode->imu->data.fused_TaitBryan[IMU_ROLL];
  theta = mode->imu->data.fused_TaitBryan[IMU_PITCH];
  psi = mode->imu->data.fused_TaitBryan[IMU_YAW];

  // get euler rates from body rates
  this->euler_rates[ROLL] = (cos(psi)/cos(theta))*mode->imu->data.gyro[IMU_ROLL] - (sin(psi)/cos(theta))*mode->imu->data.gyro[IMU_PITCH];
  this->euler_rates[PITCH] = sin(psi)*mode->imu->data.gyro[IMU_ROLL] + cos(psi)*mode->imu->data.gyro[IMU_PITCH];
  this->euler_rates[YAW] = -cos(psi)*tan(theta)*mode->imu->data.gyro[IMU_ROLL] + sin(psi)*tan(theta)*mode->imu->data.gyro[IMU_PITCH] + mode->imu->data.gyro[IMU_YAW];
}

int sign(float x){
  if( x > 0.000 ) return 1;
  else if( x < 0.000 ) return -1;
  else return 0;
}

void Control::print_attitude_rate_error(){

  this->get_attitude_rate_error();

  cout << this->error.attitude_rate_error[ROLL] << " | " << this->error.attitude_rate_error[PITCH] << " | " << this->error.attitude_rate_error[YAW] << endl;
}

void Control::print_attitude_error(){

  this->get_attitude_error();

  cout << this->error.attitude_error[ROLL] << " | " << this->error.attitude_error[PITCH] << " | " << this->error.attitude_error[YAW] << endl;
}

void flightMode::print_desired_attitude(){

  cout << this->desired_attitude[0] << " | " << this->desired_attitude[1] << " | " << this->desired_attitude[2] << endl;
}

void flightMode::print_desired_attitude_rates(){

  cout << this->desired_attitude_rates[0] << " | " << this->desired_attitude_rates[1] << " | " << this->desired_attitude_rates[2] << endl;
}
