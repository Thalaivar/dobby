#ifndef SMC_H
#define SMC_H

#include <stdint.h>
#include "pwm.h"
#include "imu.h"

#define smc_roll_lambda 0
#define smc_pitch_lambda 0
#define smc_yaw_lambda 0
#define smc_roll_eta 0
#define smc_pitch_eta 0
#define smc_yaw_eta 0

typedef enum smc_lambda{

};
class SMC{
  private:
    IMU *imu;
    Motors *motors;
    float error_roll, error_pitch, error_yaw;
    void get_angle_error();

  public:
    void run_smc_controller();
    SMC(IMU* imu_ptr, Motors* motors_ptr);
};

#endif
