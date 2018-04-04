#include "logging.h"

int Logging::log_attitude(float roll, float pitch, float yaw){

  if(fprintf(attitude_data_file, "%f,%f,%f\n", roll, pitch, yaw) < 0){
    this->log_fail_attitude++;
    return -1;
  }

  return 0;
}

int Logging::log_channel_vals(int ch1, int ch2, int ch3, int ch4){
  if(fprintf(channel_data_file, "%d,%d,%d,%d\n", ch1, ch2, ch3, ch4) < 0){
    this->log_fail_channel++;
    return -1;
  }

  return 0;
}

int Logging::log_body_rate_error(float wx, float wy, float wz){
  if(fprintf(error_data_file, "%f,%f,%f\n", wx, wy, wz) < 0){
    this->log_fail_body_rate_error++;
    return -1;
  }

  return 0;
}

int Logging::log_s(float sx, float sy, float sz){
  if(fprintf(s_data_file, "%f,%f,%f\n", sx, sy, sz) < 0){
    this->log_fail_s++;
    return -1;
  }

  return 0;
}

Logging::Logging(){

  attitude_data_file = fopen("attitude_data_file.txt", "w");
  channel_data_file  = fopen("channel_data_file.txt", "w");
  error_data_file    = fopen("error_data_file.txt", "w");
  s_data_file 		 = fopen("s_data_file.txt", "w");

  if(attitude_data_file == NULL && channel_data_file == NULL && error_data_file == NULL)
    this->is_initialised = false;
  else
    this->is_initialised = true;
}

Logging::~Logging(){

  fclose(attitude_data_file);
  fclose(channel_data_file);
  fclose(error_data_file);
}
