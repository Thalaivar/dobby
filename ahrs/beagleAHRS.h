//******************************************//
//*******		    AHRS	 LIBRARY			********//
//******************************************//


#include <roboticscape.h>
#include <beagleIMU.h>
#include <iostream>
#include <math.h>

//needs to become dynamic so that can be set from GCS
#define GYRO_GAIN_COMPLEMENTARY_FILTER 0.94
#define ACCEL_GAIN_COMPLEMENTARY_FILTER 0.06

class beagle_ahrs {

        public:

            float roll;
            float pitch;
            float yaw;
            float accel_roll;
            float accel_pitch;

            // takes in ax, ay, az, gx, gy, gz and returns roll and pitch //
            void run_complementary_filter_accel_gyro(float* accelData, float* gyroData, float* roll, float* pitch);

            // gets angles from accelerometer //
            void get_angles_accel(float* accelData)
            


    }
