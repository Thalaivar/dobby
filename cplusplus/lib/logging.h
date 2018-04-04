#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>

class Logging{
  private:
    FILE *attitude_data_file;
    FILE *channel_data_file;
    FILE *error_data_file;
	FILE *s_data_file;

  public:

    // to log attitude data
    int log_attitude(float roll, float pitch, float yaw);
	
	int log_s(float sx, float sy, float sz);

    // to log channel outputs
    int log_channel_vals(int ch1, int ch2, int ch3, int ch4);

    // to log error in body rates
    int log_body_rate_error(float wx, float wy, float wz);

    // to keep track of logging fail
    int log_fail_attitude = 0;
    int log_fail_channel = 0;
    int log_fail_body_rate_error = 0;
	int log_fail_s = 0;

    // to check if logging is ready
    bool is_initialised;

    Logging();
    ~Logging();

};
#endif
