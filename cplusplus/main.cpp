#include "dobby.h"
#include <iostream>

using namespace std;

Dobby dobby;

void exit_Handler(int a){
 	cout << endl << "Exit Handler Reached!" << endl;
  dobby.motors.is_armed = false;
  dobby.motors.disable_motors();
  exit(0);
 }


int main(){

  signal(SIGINT, exit_Handler);

  // program has started
  dobby.state = RUN;

  dobby.setup();

  dobby.pre_flight_checks();

  dobby_time current_time;

  char resp;
  short i = 0;

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
        if(i == 0){
          dobby.reset_all_times();
          i++;
         }
        current_time = timer::now();
        dobby.radio_update_loop(current_time);
        dobby.control_loop(current_time);
        dobby.motor_update_loop(current_time);
		dobby.logging_loop(current_time);
	  }
    }
	 }
  }
  return 0;
}
