#include "dobby.h"
#include <iostream>

#define N_LOOPS 10000

using namespace std;

IMU imu;

int main(){

  if(imu.init_imu() < 0)
    cout << "*********************\n* IMU initialization failed! *\n*********************";

  int count = 0;

  while(count < N_LOOPS){
    imu.update()
    cout << "Roll: " << imu.euler_angles[ROLL]*RAD_TO_DEG << "| Pitch: " << imu.euler_angles[PITCH]*RAD_TO_DEG << "| Yaw: " << imu.euler_angles[YAW]*RAD_TO_DEG << endl;
  }
}
