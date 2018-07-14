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


int main(int argc, char** argv){

  signal(SIGINT, exit_Handler);

  // program has started
  dobby.state = RUN;
  dobby_time current_time;
  char resp;
  short i = 0;

  if(opt == "pwm_test"){
    int desired_test_pwm[4] = {1000, 1000, 1000, 1000};

    if(dobby.pwm_test_setup() < 0){
      cout << "****** program halted ********" << endl;
      return 0;
    }

    if(motors.is_pru_initialized)
      dobby.state == READY_TO_FLY;

    cout << "Enter 4 PWM values: ";
    cin >> desired_test_pwm[0] >> desired_test_pwm[1] >> desired_test_pwm[2] >> desired_test_pwm[3];
    cout << "****************************\n*      Beginning IMU Test      *\n****************************" << endl;
    cout << "Enter \"y\" to continue: ";
    cin >> resp;

    if(resp == "y"){
       while(dobby.state == READY_TO_FLY){
         if(i == 0){
           dobby.reset_all_times();
           i++;
         }

         current_time = timer::now();
         dobby.pwm_test_loop(current_time, desired_test_pwm);
       }
    }
  }

  else if(opt == "imu_test"){
    if(dobby.imu_test_setup() < 0){
      cout << "****** program halted ********" << endl;
      return 0;
    }

    if(dobby.imu.is_initialized && dobby.imu.is_calibrated)
      dobby.state == READY_TO_FLY;

    cout << "****************************\n*      Beginning IMU Test      *\n****************************" << endl;
    cout << "Enter \"y\" to continue: ";
    cin >> resp;

    if(resp == "y"){
      while(dobby.state == READY_TO_FLY){
          if(i == 0){
            dobby.reset_all_times();
            i++;
          }

          current_time = timer::now();
          dobby.imu_test_update_loop(current_time);
          dobby.imu_test_logging_loop(current_time);
      }
    }
  }

  else if(opt == "run"){
    dobby.setup();

    dobby.pre_flight_checks();

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
}
