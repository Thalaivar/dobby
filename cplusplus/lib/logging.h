#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>

class Logging{
  private:
    FILE *data_file;

  public:
    
	int log_3_axis_data(float ax, float ay, float az);

    // to keep track of logging fail
    int log_fail_3_axis = 0;

    // to check if logging is ready
    bool is_initialised;
    
    Logging();
    ~Logging();

};
#endif
