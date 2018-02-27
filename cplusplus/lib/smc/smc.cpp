#include "smc.h"

SMC::SMC(IMU* imu_ptr, Motors* motors_ptr){

  // link imu and motors classes to smc controller
  this->imu = imu_ptr;
  this->motors = motors_ptr;

}
void SMC::run_smc_controller(){

  
}
