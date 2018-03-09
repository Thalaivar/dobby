#ifndef DOBBY_H
#define DOBBY_H

#include <roboticscape.h>
#include <iostream>
#include "ppm.h"
#include "pwm.h"
#include "control.h"
#include "imu.h"

// to keep track of dobby's state

/**********************************************************************************
  * READY_TO_FLY = All peripherals are ready and enabled, arming left
  * FLYING = Currently in air
  * ARMED = State before FLYING, can only be set if previous state was READY_TO_FLY
  * DISARMED = State usually after flying, device still ready for take off
  * NOT_READY = pre flight checks (and others?) need to be done
/**********************************************************************************/
typedef enum dobby_status{
  READY_TO_FLY = 0,
  FLYING,
  ARMED,
  DISARMED,
  NOT_READY
}dobby_status;

/***********************************************************
                      main dobby class
***********************************************************/
class Dobby{
  private:

    // define all dobby peripherals
    IMU imu;
    Receiver recv;
    Motors motors = Motors(&recv);
    flightMode mode = flightMode(&recv, &imu);
    Control control = Control(&motors, &mode);

    // holds current status of dobby
    dobby_status state;

  public:

    // pre flight checks
    int pre_flight_checks();

    // runs on separate thread, keeps checking for disarm signal
    int disarm_check();

    // setup function
    void setup();

    // main loop
    
    Dobby();
}
#endif
