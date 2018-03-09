#include "dobby.h"

int Dobby::pre_flight_checks(){

  // check current state
  if(this->state == READY_TO_FLY){
    cout << "DOBBY IS READY!";
    return 0;
  }

  // check IMU
  if(!imu.is_initialized){
    cout << "IMU not initialized!\n";
    return -1;
  }

  // check Receiver
  if(!recv.is_initialized){
    cout << "Receiver not initialized!\n";
    return -1;
  }

  // check if flight mode is set
  if(mode.current_mode == NOT_SET){
    cout << "Flight mode not set, cant take off!\n";
    return -1;
  }

  // any checks for controller?

  // check Motors
  if(!motors.is_initialized){
    cout << "Motors not initialized\n";
    return -1;
  }


  // if everything checks out, ready to fly!
  cout << "DOBBY IS READY!";
  this->status = READY_TO_FLY;
  return 0;
}
