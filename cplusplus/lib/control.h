#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <cmath>
#include "pwm.h"
#include "imu.h"
#include "ppm.h"

/***************************************************************
                            smc params
***************************************************************/
#define smc_roll_lambda  0.0
#define smc_pitch_lambda 1
#define smc_yaw_lambda   0.0

#define smc_pitch_k      1
#define smc_roll_eta     0.0
#define smc_pitch_eta    0
#define smc_yaw_eta      0.0

/***************************************************************
						PID Params
***************************************************************/
#define pid_roll_kp 	0.00
#define pid_pitch_kp	24.00
#define pid_yaw_kp 		0.00

#define pid_roll_ki 	0.00
#define pid_pitch_ki	0.15
#define pid_yaw_ki 		0.00

#define pid_roll_kd 	0.00
#define pid_pitch_kd	4.75
#define pid_yaw_kd 		0.00

/***************************************************************
              recv signal conversion to desired values
***************************************************************/
#define recv_signal_to_roll_angle 0.119f
#define recv_signal_to_pitch_angle 0.111f
#define recv_signal_to_yaw_angle 0.17f

/***************************************************************
          angle error to rate error conversion params
***************************************************************/
#define angle_to_rate_roll  3
#define angle_to_rate_pitch 3
#define angle_to_rate_yaw   3

/***************************************************************
				integral windup limiters
***************************************************************/
#define INTG_WNDP_ROLL_POS 7.0f
#define INTG_WNDP_ROLL_NEG -7.0f
#define INTG_WNDP_PITCH_POS 10.0f
#define INTG_WNDP_PITCH_NEG -6.0f

/***************************************************************
                    quadcopter dynamics params
***************************************************************/
//              battery  +    ESCs    +  motors   +   frame
#define Ixx 1000*(0.000081599 + 0.00032233 + 0.0133609 + 0.0044966)
#define Iyy 1000*(0.000586590 + 0.00032233 + 0.0133609 + 0.0044966)
#define Izz 1000*(0.000621150 + 0.00032968 + 0.0163521 + 0.0118809)

#define k1 -10.41
#define k3 10.92
#define k5 -0.51


#define LOOP_TIME 0.00503
#define INV_LOOP_TIME 200
#define PI_BY_2_INV 0.6363
#define PID_PITCH_CUTOFF 1.22173

// forward declaration to avoid error
class flightMode;

struct error_struct{
  float d_body_rate_error[3]; // Derivative of Body rate Error
  float ie_body_rate[3]; // integral body rate error
  float body_rate_error[3];
  float attitude_error[3];
  float prev_body_rate_error[3] = {0,0,0};
};

struct pid_struct{
  float pid_p, pid_d, pid_i;
};

class Control{
  private:
    Motors *motors;
    flightMode *mode;

    // to access IMU data
    IMU *imu;

    // to get attitude_rate_error
    void get_body_rate_error();

    // to convert control signals to torques (to be sent to motors)
    void demux_control_signal();

    // get desired body rates from desired euler rates
    void get_desired_body_rates();

    // get desired euler rates from current euler angle error
    void get_desired_euler_rates();

    // get latest integral error
    void get_ie_body_rate_error();

  public:
    void run_smc_controller();
  	void run_pid_controller();


    float desired_body_rates[3];
    float desired_euler_rates[3];
    float u_phi, u_theta, u_psi;
	  float s_roll, s_pitch, s_yaw;

    // for PID controller
    pid_struct phi_pid;
    pid_struct theta_pid;
    pid_struct psi_pid;

    error_struct error;


	Control(Motors* motors_ptr, flightMode* flightMode_ptr, IMU* imu_ptr);
};

typedef enum flight_modes{
   STABILIZE_ANGLE = 0,
   STABILIZE_RATES,
   ONE_DOF_TEST,
   NOT_SET,
   FAIL
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

    // current desired euler angles
	  float desired_euler[3];

  private:
    // to access the receiver signals
    Receiver *recv;

    // to access imu data
    IMU *imu;

    // controller needs access to desired trajectory/attitude
    friend class Control;
};

int sign(float x);
#endif
