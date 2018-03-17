#ifndef SMC_H
#define SMC_H

#include <stdint.h>
#include <cmath>
#include "pwm.h"
#include "imu.h"
#include "ppm.h"
#include <time.h>

/***************************************************************
                            smc params
***************************************************************/
#define smc_roll_lambda  1
#define smc_pitch_lambda 1
#define smc_yaw_lambda   1
#define smc_roll_eta     1
#define smc_pitch_eta    1
#define smc_yaw_eta      1

/***************************************************************
              recv signal conversion to desired values
***************************************************************/
#define recv_signal_to_roll_angle 0.1f
#define recv_signal_to_pitch_angle 0.07f
#define recv_signal_to_yaw_angle 0.07f

/***************************************************************
          angle error to rate error conversion params
***************************************************************/
#define angle_to_rate_roll 0.02
#define angle_to_rate_pitch 0.02
#define angle_to_rate_yaw 0.02

/***************************************************************
                    quadcopter dynamics params
***************************************************************/
//              battery  +    ESCs    +  motors   +   frame
#define Ixx 100*(0.000081599 + 0.00032233 + 0.0133609 + 0.0044966)
#define Iyy 100*(0.000586590 + 0.00032233 + 0.0133609 + 0.0044966)
#define Izz 100*(0.000621150 + 0.00032968 + 0.0163521 + 0.0118809)
#define thrust_coeff 0
#define drag_coeff 0

// forward declaration to avoid error
class flightMode;

struct error_struct{
  // integral body rate error
  float ie_body_rate[3];
  float body_rate_error[3];
};

clock_t t;

class Control{
  private:
    Motors *motors;
    flightMode *mode;

    // to access IMU data
    IMU *imu;
    error_struct error;

    clock_t prev_time;

    // holds outputs of controller
    float control_signal[4];

    // to get attitude_rate_error
    void get_body_rate_error();

    // to convert control signals to torques (to be sent to motors)
    void demux_control_signal();

    // get desired body rates from desired euler rates
    void get_desired_body_rates();

  public:
    void run_smc_controller();

    // debug functions , can be deleted later
    void print_body_rate_error();

    // desired body rates
    float desired_body_rates[3];

    Control(Motors* motors_ptr, flightMode* flightMode_ptr, IMU* imu_ptr);
};

typedef enum flight_modes{
   STABILIZE_ANGLE = 0,
   STABILIZE_RATES,
   NOT_SET
}flight_mode;

class flightMode{
  public:
    flightMode(Receiver *recv_ptr, IMU *imu_ptr);
    // to set current flight mode
    void set_flight_mode(flight_mode desired_mode);

    // to get latest desired trajectory/attitude
    void flight_mode_update();

    // holds current mode setting
    flight_mode current_mode;

    // debug functions, can be deleted angle_to_rate_roll
    void print_desired_attitude();
    void print_desired_attitude_rates();

  private:
    // to access the receiver signals
    Receiver *recv;

    // to access imu data
    IMU *imu;

    // holds latest desired values
    float desired_euler[3];
    float desired_euler_rates[3];

    // controller needs access to desired trajectory/attitude
    friend class Control;
};

int sign(float x);
#endif
