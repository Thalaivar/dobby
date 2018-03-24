#ifndef DOBBY_H
#define DOBBY_H

#include <roboticscape.h>
#include <iostream>
#include <chrono>
#include <ratio>
#include "ppm.h"
#include "pwm.h"
#include "control.h"
#include "imu.h"
#include "logging.h"

#define FASTLOOP_PERIOD 5000
#define RADIO_LOOP_PERIOD 20000
#define MOTOR_LOOP_PERIOD 20000

using namespace std;

// to keep track of dobby's state

/**********************************************************************************
  * READY_TO_FLY = All peripherals are ready and enabled, arming left
  * FLYING = Currently in air
  * ARMED = State before FLYING, can only be set if previous state was READY_TO_FLY
  * DISARMED = State usually after flying, device still ready for take off
  * NOT_READY_TO_FLY = pre flight checks (and others?) need to be done
  * RUNNING = main program execution has begun, dobby's state is set to this at the very start
  * EXITING = main program execution over, set when all flying is over
**********************************************************************************/
typedef enum dobby_status{
  READY_TO_FLY = 0,
  FLYING,
  ARMED,
  DISARMED,
  NOT_READY_TO_FLY,
  RUN,
  EXIT
}dobby_status;

typedef chrono::high_resolution_clock timer;
typedef chrono::high_resolution_clock::time_point dobby_time;
typedef chrono::microseconds us;
typedef std::chrono::duration<double> loop_time;

struct loop_times{

  double fast_loop_time;
  double radio_loop_time;
  double motor_loop_time;
  double logging_loop_time;
  dobby_time fast_loop_prev_time;
  dobby_time radio_loop_prev_time;
  dobby_time motor_loop_prev_time;
  dobby_time logging_loop_prev_time;
};

/***********************************************************
                      main dobby class
***********************************************************/
class Dobby{
  private:
    loop_times times;

  public:
float avg_time = 0;
long count = 0;
long count1 = 0;
    // define all dobby peripherals
    IMU imu;
    Receiver radio;
    Motors motors = Motors(&radio);
    flightMode mode = flightMode(&radio, &imu);
    Control control = Control(&motors, &mode, &imu);\
	Logging logging;

    // pre flight checks
    int pre_flight_checks();

    // runs on separate thread, keeps checking for disarm signal,
    // once signal is received, disables motors (and any other things??)
    int disarm_check();

    // setup function
    int setup();

    // main loop
    void control_loop(dobby_time current_time);

    void radio_update_loop(dobby_time current_time);

    void motor_update_loop(dobby_time current_time);
	
	void logging_loop(dobby_time current_time);

    // to reset all times for the different loops, to be called just before FLYING
    void reset_all_times();

    // holds current status of dobby
    dobby_status state;
};
#endif
