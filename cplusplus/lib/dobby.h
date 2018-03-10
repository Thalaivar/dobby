#ifndef DOBBY_H
#define DOBBY_H

#include <roboticscape.h>
#include <iostream>
#include "ppm.h"
#include "pwm.h"
#include "control.h"
#include "imu.h"

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
/**********************************************************************************/
typedef enum dobby_status{
  READY_TO_FLY = 0,
  FLYING,
  ARMED,
  DISARMED,
  NOT_READY_TO_FLY,
  RUNNING,
  EXITING
}dobby_status;

/***********************************************************
                      main dobby class
***********************************************************/
class Dobby{
  private:

    // define all dobby peripherals
    IMU imu;
    Receiver radio;
    Motors motors = Motors(&radio);
    flightMode mode = flightMode(&radio, &imu);
    Control control = Control(&motors, &mode);

    // holds current status of dobby
    dobby_status state;

  public:

    // pre flight checks
    int pre_flight_checks();

    // runs on separate thread, keeps checking for disarm signal,
    // once signal is received, disables motors (and any other things??)
    int disarm_check();

    // setup function
    int setup();

    // main loop
    void control_loop();


}
#endif
