#include "logging.h"

int Logging::log_3_axis_data(float ax, float ay, float az){

  if(fprintf(data_file, "%f,%f,%f\n", ax, ay, az) < 0){
    this->log_fail_3_axis++;
    return -1;
  }

  return 0;
}

Logging::Logging(){

  data_file = fopen("data_file.txt", "w");

  if(data_file == NULL)
    this->is_initialised = false;
  else
    this->is_initialised = true;
}

Logging::~Logging(){

  fclose(data_file);
}
