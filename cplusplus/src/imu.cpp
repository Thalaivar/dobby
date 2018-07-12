#include "imu.h"

IMU::IMU(void){

    config.gyro_fsr =  GYRO_FSR_1000DPS;
    config.accel_fsr =  ACCEL_FSR_4G;
    config.gyro_dlpf = GYRO_DLPF_92;
    config.accel_dlpf = ACCEL_DLPF_92;
	config.i2c_bus = I2C_BUS;
	config.i2c_addr = I2C_ADDR;
    config.enable_magnetometer = 1;
    config.dmp_fetch_accel_gyro = 0;
	config.dmp_auto_calibrate_gyro = 1;
    config.dmp_sample_rate = 200;
    config.orient = ORIENTATION_Z_UP;
    config.compass_time_constant = 2.0;
    config.dmp_interrupt_priority = 0;
    config.show_warnings = 0;
    config.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
    config.gpio_interrupt_pin = GPIO_INT_PIN_PIN;

    this->is_initialized = false;
    this->is_calibrated = false;
}


int IMU::init_imu(){

    if(this->is_initialized){
      cerr << "IMU already initialized!" << '\n';
      return 0;
    }

    if(rc_mpu_is_gyro_calibrated() == 0){
      printf("Gyro needs calibration!\nRunning gyro calibration routine....\n");
      if(rc_mpu_calibrate_gyro_routine(config) < 0) {
        printf("Gyro calibration failed!\n");
        return -1;
      }
    }

    if(rc_mpu_is_mag_calibrated() == 0){
      printf("Mag needs calibration!\nRunning gyro calibration routine....\n");
      if(rc_mpu_calibrate_mag_routine(config) < 0) {
        printf("Mag calibration failed!\n");
        return -1;
      }
    }

	if(rc_mpu_is_accel_calibrated() == 0){
      printf("Accel needs calibration!\nRunning gyro calibration routine....\n");
      if(rc_mpu_calibrate_accel_routine(config) < 0) {
        printf("Accel calibration failed!\n");
        return -1;
      }
    }
    if(rc_mpu_is_mag_calibrated() == 1 && rc_mpu_is_gyro_calibrated() == 1 && rc_mpu_is_accel_calibrated() == 1){
      this->is_calibrated = true;
    }

    if(rc_mpu_initialize_dmp(&data, config) < 0){
      printf("DMP init failed!\n");
      return -1;
    }

    this->is_initialized = true;
    return 0;
}

void IMU::update(){

  // populate euler angles with latest data in rad
  euler_angles[ROLL]  = data.fused_TaitBryan[ROLL];
  euler_angles[PITCH] = data.fused_TaitBryan[PITCH];
  euler_angles[YAW]   = data.fused_TaitBryan[YAW];

  // populate body rates with latest data in rad/s
  body_rates[ROLL]  = data.gyro[ROLL]*DEG_TO_RAD;
  body_rates[PITCH] = data.gyro[PITCH]*DEG_TO_RAD;
  body_rates[YAW]   = data.gyro[YAW]*DEG_TO_RAD;

}
