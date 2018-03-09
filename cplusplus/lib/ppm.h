#ifndef PPM_H
#define PPM_H

#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>

#define PRU_CYCLES_TO_US 0.01
#define PPM_PRU 0

/***************************************************************
                          channel nos
***************************************************************/
#define ROLL_CHANNEL 0
#define PITCH_CHANNEL 1
#define YAW_CHANNEL 3
#define THROTTLE_CHANNEL 2

using namespace std;

typedef uint32_t u32;

struct recv_channelPtr {
  u32 ch1;
  u32 ch2;
  u32 ch3;
  u32 ch4;
  u32 ch5;
  u32 ch6;
};

class Receiver {
  private:
    recv_channelPtr p;
    recv_channelPtr volatile *channels;

    // calibration stuff
    int save_radio_cal();
    int load_radio_cal();

  public:
    int initialize_pru();
    int disable_pru();
    int update();
    u32 recv_channel[6];
    bool is_pru_initialized;
    bool is_radio_initialized;
    bool is_calibrated;
    // calibration holds the mean value (to subtract when caculating setpoints)
    int cal_roll, cal_pitch, cal_yaw, cal_throttle;

    int init_radio();
    // calibration currently calculates the appropriate mean value (assuming linear)
    int calibrate_radio();
    Receiver();
};
#endif
