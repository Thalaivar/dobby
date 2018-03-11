#include "dobby.h"

Dobby dobby;

int main(){

  // program has started
  dobby.state = RUN;

  dobby.setup();

  dobby.pre_flight_checks();

  while(dobby.state == READY_TO_FLY){
    if(dobby.motors.arm_motors()) dobby.state = ARMED;
    while(dobby.state == ARMED){
      imu.print_tb_angles();
    }

  }
  return 0;
}
