#include "smc.h"

SMC::SMC(IMU* imu_ptr, Motors* motors_ptr){

  // link imu and motors classes to smc controller
  this->imu = imu_ptr;
  this->motors = motors_ptr;

}


flightMode::flightMode(Receiver *recv_ptr){

  // link reciever class to get latest reciever data
  this->recv = recv_ptr;
}

void flightMode::get_error(error_struct *error){

  // check what current flight mode is
  switch(this->current_mode){

    case STABILIZE_ANGLE:

  }
}

void flightMode::flight_mode_update(){

  // check what current flight mode is
  switch(this->current_mode){

    case STABILIZE_ANGLE:
    // get desired angles from pilot in degrees
      this->desired_attitude[0] = (recv->recv_channel[ROLL_CHANNEL] - recv->cal_roll)*recv_signal_to_roll;
      this->desired_attitude[1] = (recv->recv_channel[PITCH_CHANNEL] - recv->cal_pitch)*recv_signal_to_pitch;
      this->desired_attitude[2] = (recv->recv_channel[YAW_CHANNEL] - recv->cal_yaw)*recv_signal_to_yaw;

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
