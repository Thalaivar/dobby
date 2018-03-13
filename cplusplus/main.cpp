#include "dobby.h"
#include <iostream>

using namespace std;

Dobby dobby;

int main(){

  // program has started
  dobby.state = RUN;

  dobby.setup();

  dobby.pre_flight_checks();

  char resp;

  cout << dobby.state << endl;
  cout << "Enter \"y\" to continue: ";
  cin >> resp;

  if(resp == 'y'){
    while(dobby.state == READY_TO_FLY){
    dobby.radio.update();
	  dobby.motors.arm_motors();
	  if(dobby.motors.is_armed) dobby.state = ARMED;
	  while(dobby.state == ARMED){
    if(dobby.radio.recv_channel[2] > 1200) dobby.state == FLYING;
     while(dobby.state == FLYING){
       dobby.radio.update();
       dobby.motors.channel_val[0] = dobby.radio.recv_channel[2];
	     dobby.motors.channel_val[1] = dobby.radio.recv_channel[2];
       dobby.motors.channel_val[2] = dobby.radio.recv_channel[2];
       dobby.motors.channel_val[3] = dobby.radio.recv_channel[2];
       dobby.motors.update();
      }
    }
	 }
  }
  return 0;
}
