#ifndef SMC_H
#define SMC_H

#include <stdint.h>
#include <cmath>
#include "pwm.h"
#include "imu.h"
#include "ppm.h"

/***************************************************************
                            smc params
***************************************************************/
#define smc_roll_lambda 0
#define smc_pitch_lambda 0
#define smc_yaw_lambda 0
#define smc_roll_eta 0
#define smc_pitch_eta 0
#define smc_yaw_eta 0

/***************************************************************
              recv signal conversion to desired values
***************************************************************/
#define recv_signal_to_roll_angle 0.07f
#define recv_signal_to_pitch_angle 0.07f
#define recv_signal_to_yaw_angle 0.07f

/***************************************************************
          angle error to rate error conversion params
***************************************************************/
#define angle_to_rate_roll 0
#define angle_to_rate_pitch 0
#define angle_to_rate_yaw 0

/***************************************************************
                    quadcopter dynamics params
***************************************************************/
#define Ixx 0
#define Iyy 0
#define Izz 0
#define thrust_coeff 0
#define drag_coeff 0

// forward declaration to avoid error
class flightMode;

struct error_struct{
  float attitude_error[3];
  float attitude_rate_error[3];
};

typedef enum attribute {
  ROLL = 0,
  PITCH,
  YAW
}attirubute;

class Control{
  private:
    Motors *motors;
    flightMode *mode;
    error_struct error;

    // holds outputs of controller
    float control_signal[4];

    // angle rates in euler frames
    float euler_rates[3];

    // to get attitude error
    void get_attitude_error();

    // to get attitude_rate_error
    void get_attitude_rate_error();

    // to convert control signals to torques (to be sent to motors)
    void demux_control_signal();

    // body rates to euler rates
    void body_to_euler_rates();

  public:
    void run_smc_controller();

    // debug functions , can be deleted later
    void print_attitude_error();
    void print_attitude_rate_error();
    
    Control(Motors* motors_ptr, flightMode* flightMode_ptr);
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
    float desired_attitude[3];
    float desired_attitude_rates[3];

    // controller needs access to desired trajectory/attitude
    friend class Control;
};

int sign(float x);
#endif
