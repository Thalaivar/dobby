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
#define recv_signal_to_roll 0
#define recv_signal_to_pitch 0
#define recv_signal_to_yaw 0

/***************************************************************
          angle error to rate error conversion params
***************************************************************/
#define angle_to_rate_roll 0
#define angle_to_rate_pitch 0
#define angle_to_rate_yaw 0

typedef struct error_struct{
  float attitude_error[3];
  float attitude_rate_error[3];
};

class SMC{
  private:
    IMU *imu;
    Motors *motors;
    error_struct error;

    // allow error calculation function to access error variables

  public:
    void run_smc_controller();
    SMC(IMU* imu_ptr, Motors* motors_ptr);
};

typedef enum flight_modes{
   STABILIZE_ANGLE = 0,
   STABILIZE_RATES,
   INCORRECT
}flight_mode;

class flightMode{
  public:
    void get_error(error_struct *error);
    flightMode(Receiver *recv_ptr);

  private:
    // to access the receiver signals
    Receiver *recv;

    // holds current mode setting
    flight_mode current_mode;

    // holds latest desired values
    float desired_attitude[3];
    float desired_attitude_rates[3];

    void set_flight_mode();
    void flight_mode_update();

}
#endif
