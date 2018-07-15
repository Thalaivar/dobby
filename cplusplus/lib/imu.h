/**************************************************
                imu library
***************************************************/

#ifndef IMU_H
#define IMU_H

#include <robotcontrol.h>
#include <getopt.h>
#include <signal.h>		// capture ctrl-c
#include <pthread.h>	// multi-threading
#include <iostream>

using namespace std;

#define ROLL 0
#define PITCH 1
#define YAW 2
#define I2C_BUS 2
#define I2C_ADDR 0x68
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN 21

class IMU{
  public:
    rc_mpu_config_t config;
    rc_mpu_data_t   data;

    // euler angles in radians
    float euler_angles[3];

    // body rates in rad/s
    double body_rates[3];

	  IMU();

	  int init_imu();

    // call this to get latest euler angles
    void update();


    bool is_initialized;
    bool is_calibrated;
};
#endif
