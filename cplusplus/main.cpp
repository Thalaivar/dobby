#include "dobby.h"
#include <iostream>

using namespace std;

Dobby dobby;

void exit_Handler(int a){
 	cout << endl << "Exit Handler Reached!" << endl;
	dobby.motors.disable_motors();
	exit(0);
 }


int main(){

  signal(SIGINT, exit_Handler);

  // program has started
  dobby.state = RUN;

  dobby.setup();

  dobby.pre_flight_checks();

  char resp;

  cout << "Enter \"y\" to continue: ";
  cin >> resp;

  if(resp == 'y'){
    while(dobby.state == READY_TO_FLY){
    dobby.radio.update();
	  dobby.motors.arm_motors();
	  if(dobby.motors.is_armed) dobby.state = ARMED;
	  while(dobby.state == ARMED){
	  dobby.radio.update();
      if(dobby.radio.recv_channel[2] > 1200) dobby.state = FLYING;
     	while(dobby.state == FLYING){
			dobby.imu.print_tb_angles();
			//dobby.control_loop(); 
			//std::this_thread::sleep_for(std::chrono::milliseconds(100));
	   }
      }
	 }
  }
  return 0;
}
