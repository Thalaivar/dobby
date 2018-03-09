#ifndef SMC_H
#define SMC_H

#include <stdint.h>
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
#define recv_signal_to_roll_angle 0
#define recv_signal_to_pitch_angle 0
#define recv_signal_to_yaw_angle 0

/***************************************************************
          angle error to rate error conversion params
***************************************************************/
#define angle_to_rate_roll 0
#define angle_to_rate_pitch 0
#define angle_to_rate_yaw 0

struct error_struct{
  float attitude_error[3];
  float attitude_rate_error[3];
};

class Control{
  private:
    Motors *motors;
    flightMode *mode;
    error_struct error;

    // to get attitude error
    void get_attitude_error();

    // to get attitude_rate_error
    void get_attitude_rate_error();

  public:
    void run_smc_controller();
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

  private:
    // to access the receiver signals
    Receiver *recv;

    // to access imu data
    IMU *imu;

    // holds current mode setting
    flight_mode current_mode;

    // holds latest desired values
    float desired_attitude[3];
    float desired_attitude_rates[3];

    // controller needs access to desired trajectory/attitude
    friend class Control;
};
#endif
