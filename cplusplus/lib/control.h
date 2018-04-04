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
#define smc_roll_lambda  1.024
#define smc_pitch_lambda 1.324
#define smc_yaw_lambda   0.324
#define smc_roll_eta     1.6
#define smc_pitch_eta    2.0
#define smc_yaw_eta      1.6

/***************************************************************
              recv signal conversion to desired values
***************************************************************/
#define recv_signal_to_roll_angle 0.119f
#define recv_signal_to_pitch_angle 0.111f
#define recv_signal_to_yaw_angle 0.17f

/***************************************************************
          angle error to rate error conversion params
***************************************************************/
#define angle_to_rate_roll -3.8
#define angle_to_rate_pitch -3.8
#define angle_to_rate_yaw -3.8

/***************************************************************
                    quadcopter dynamics params
***************************************************************/
//              battery  +    ESCs    +  motors   +   frame
#define Ixx 1000*(0.000081599 + 0.00032233 + 0.0133609 + 0.0044966)
#define Iyy 1000*(0.000586590 + 0.00032233 + 0.0133609 + 0.0044966)
#define Izz 1000*(0.000621150 + 0.00032968 + 0.0163521 + 0.0118809)

#define LOOP_TIME 0.00503

#define PI_BY_2_INV 0.6363

// forward declaration to avoid error
class flightMode;

struct error_struct{
  // integral body rate error
  float ie_body_rate[3];
  float body_rate_error[3];
};


class Control{
  private:
    Motors *motors;
    flightMode *mode;

    // to access IMU data
    IMU *imu;

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

    // desired body rates
    float desired_body_rates[3];
	
	float s_roll, s_pitch, s_yaw;

    error_struct error;

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

  private:
    // to access the receiver signals
    Receiver *recv;

    // to access imu data
    IMU *imu;

    // holds latest desired values
    float desired_euler[3];
	  float desired_euler_rotated[3];

	// to make sure angles are in terms of what the pilot "sees"
    void rotate_desired_euler_angles();

    // controller needs access to desired trajectory/attitude
    friend class Control;
};

int sign(float x);
#endif
