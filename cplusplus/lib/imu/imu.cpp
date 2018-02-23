#include "imu.h"

IMU::IMU(void){

    config.gyro_fsr = G_FSR_500DPS;
    config.accel_fsr = A_FSR_4G;
    config.gyro_dlpf = GYRO_DLPF_92;
    config.accel_dlpf = ACCEL_DLPF_92;

    config.enable_magnetometer = 1;

    config.dmp_sample_rate = 200;
    config.orientation = ORIENTATION_Z_UP;
    config.compass_time_constant = 2.0;
    config.dmp_interrupt_priority = sched_get_priority_max(SCHED_FIFO)-1;
    config.show_warnings = 0;
}


int IMU::init_imu(){
    if(rc_is_gyro_calibrated() == 0){
      printf("Gyro needs calibration!\nRunning gyro calibration routine....\n");
      if(rc_calibrate_gyro_routine() < 0) printf("Gyro calibration failed!\n");
    }

    if(rc_is_mag_calibrated() == 0){
      printf("Mag needs calibration!\nRunning gyro calibration routine....\n");
      if(rc_calibrate_mag_routine() < 0) printf("Mag calibration failed!\n");
    }

    if(rc_initialize_imu_dmp(&data, config) < 0) printf("DMP init failed!\n");
}

void IMU::print_tb_angles(){
    printf("Roll: %f | Pitch: %f | Yaw: %f\n", data.fused_TaitBryan[TB_ROLL_Y]*RAD_TO_DEG, data.fused_TaitBryan[TB_PITCH_X]*RAD_TO_DEG\
                                               , data.fused_TaitBryan[TB_YAW_Z]*RAD_TO_DEG);
}
