#include "Control.h"

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
    // get desired angles from pilot in degrees
      this->desired_attitude[0] = (recv->recv_channel[ROLL_CHANNEL] - recv->cal_roll)*recv_signal_to_roll_angle;
      this->desired_attitude[1] = (recv->recv_channel[PITCH_CHANNEL] - recv->cal_pitch)*recv_signal_to_pitch_angle;
      this->desired_attitude[2] = (recv->recv_channel[YAW_CHANNEL] - recv->cal_yaw)*recv_signal_to_yaw_angle;

    // get desired angular rates (by passing through simple P controller)
      this->desired_attitude_rates[0] = (imu->data.fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG - this->desired_attitude[0])*angle_to_rate_roll;
      this->desired_attitude_rates[1] = (imu->data.fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG - this->desired_attitude[1])*angle_to_rate_pitch;
      this->desired_attitude_rates[2] = (imu->data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG - this->desired_attitude[2])*angle_to_rate_yaw;
      break;

    default:
      cout << "INCORRECT FLIGHT MODE!!";
      this->current_mode = INCORRECT;
      break;
  }
}

void Control::get_attitude_error(){
  // <angle>_error = current_<angle> - desired_<angle>
  error.attitude_error[0] = mode->imu->data.fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG - mode->desired_attitude[0];
  error.attitude_error[1] = mode->imu->data.fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG - mode->desired_attitude[1];
  error.attitude_error[2] = mode->imu->data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG - mode->desired_attitude[2];
}

void Control::get_attitude_rate_error(){
  // <angle>_rate_error = current_<angle>_rate - desired_<angle>_rate
  error.attitude_rate_error[0] = mode->imu->data.gyro[0] - mode->desired_attitude_rates[0];
  error.attitude_rate_error[1] = mode->imu->data.gyro[1] - mode->desired_attitude_rates[1];
  error.attitude_rate_error[2] = mode->imu->data.gyro[12 - mode->desired_attitude_rates[2];
}

void Control::run_smc_controller(){

  // run standard smc if mode is simple stabilise mode with pilot inputs as angles
  if(mode->current_mode == STABILIZE_ANGLE){

  }
}
