#include "logging.h"

int Logging::log_attitude(float roll, float pitch, float yaw){

  if(fprintf(attitude_data_file, "%f,%f,%f\n", roll, pitch, yaw) < 0){
    this->log_fail_attitude++;
    return -1;
  }

  return 0;
}

int Logging::log_desired_attitude(float roll, float pitch, float yaw){

  if(fprintf(desired_attitude_data_file, "%f,%f,%f\n", roll, pitch, yaw) < 0){
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

int Logging::log_ie_body_rate_error(float ie_wx, float ie_wy, float ie_wz){
  if(fprintf(ie_data_file, "%f,%f,%f\n", ie_wx, ie_wy, ie_wz) < 0){
    this->log_fail_ie_body_rate_error++;
    return -1;
  }

  return 0;
}

int Logging::log_attitude_error(float ex, float ey, float ez){
  if(fprintf(attitude_error_data_file, "%f,%f,%f\n", ex, ey, ez) < 0){
    this->log_fail_attitude_error++;
    return -1;
  }

  return 0;
}

Logging::Logging(){

  attitude_data_file = fopen("attitude_data_file.txt", "w");
  channel_data_file  = fopen("channel_data_file.txt", "w");
  error_data_file    = fopen("error_data_file.txt", "w");
  s_data_file 		   = fopen("s_data_file.txt", "w");
  ie_data_file       = fopen("ie_data_file.txt", "w");
  attitude_error_data_file = fopen("attitude_error_data_file.txt", "w");
  desired_attitude_data_file = fopen("desired_attitude_data_file.txt", "w");
  
  if(attitude_data_file == NULL && channel_data_file == NULL && error_data_file == NULL \
     && s_data_file == NULL && ie_data_file == NULL && attitude_error_data_file == NULL)
    this->is_initialised = false;
  else
    this->is_initialised = true;
}

Logging::~Logging(){

  fclose(attitude_data_file);
  fclose(channel_data_file);
  fclose(error_data_file);
  fclose(s_data_file);
  fclose(ie_data_file);
  fclose(attitude_error_data_file);
  fclose(desired_attitude_data_file);
}
