#include "dobby.h"

IMU imu;
Logging logger;

void exit_Handler(int a){
 	cout << endl << "Exit Handler Reached!" << endl;
  dobby.motors.is_armed = false;
  dobby.motors.disable_motors();
  exit(0);
 }


int main(){

	signal(SIGINT, exit_Handler);

	if(imu.init_imu() < 0){
		cout << "IMU init failed!!!" << endl;
	}
  
    dobby_time current_time;

	else{
		
		cout << imu.is_initialized << endl;
		int count = 0;
		while(count <= 10000){
		  if(count == 0){
           	dobby.reset_all_times();
          	i++;
          }
          
		  current_time = timer::now();

		  imu.update();
		  
		  if	
//			cout << "Roll: " << imu.euler_angles[ROLL]*RAD_TO_DEG << "Pitch: " << imu.euler_angles[PITCH]*RAD_TO_DEG << "Yaw: " << imu.euler_angles[YAW]*RAD_TO_DEG << endl;
		  count++;
		}
	}

	return 0;
}
