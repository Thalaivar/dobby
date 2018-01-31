#include "ARM_IMU.h"


void armIMU::set_configuration_default(rc_imu_config_t* config){

		rc_set_imu_config_to_defaults(config);
	}

void armIMU::update_random(rc_imu_data_t* data){

		// some checks in here??
		rc_read_accel_data(data);
		rc_read_gyro_data(data);
		rc_read_mag_data(data);
	}

void armIMU::init_random(rc_imu_data_t* data, rc_imu_config_t config){

		//check if init was successful
		printf("Initializing IMU for random mode operation....\n");
		rc_initialize_imu(data, config);
		printf("IMU initialized for random mode!\n");

	}

void armIMU::enable_mag(rc_imu_config_t* config){

		config->enable_magnetometer = 1;

	}

void armIMU::print_config(rc_imu_config_t* config){

	//get the accel scales
		printf("The accel full scale range is:	");
		switch(config->accel_fsr)
			{
				case A_FSR_2G:
					printf("2G\n");
					break;

				case A_FSR_4G:
					printf("4G\n");
					break;

				case A_FSR_8G:
					printf("8G\n");
					break;

				case A_FSR_16G:
					printf("16G\n");
					break;
			}
 //get the gyro scale
			printf("The gyro full scale range is:	");
			switch(config->gyro_fsr)
				{
					case G_FSR_250DPS:
						printf("250DPS\n");
						break;

					case G_FSR_500DPS:
						printf("500DPS\n");
						break;

					case G_FSR_1000DPS:
						printf("1000DPS\n");
						break;

					case G_FSR_2000DPS:
						printf("2000DPS\n");
						break;
				}
//check if magnetometer is enabled
				if(config->enable_magnetometer){
						printf("Magnetometer enabled!\n");
				}

				else printf("Magnetometer not enabled\n");


	}

void armIMU::set_operation_mode(imu_mode* mode, imu_mode choice){

				//check if mode chosen is appropriate
				if(choice == RANDOM_MODE || choice == DMP_MODE){
							mode = choice;
					}

				else printf("Invalid imu mode chosen!\n");

	}

void debug_print_vals(rc_imu_data_t* data){

			printf("Accel: [	%f	%f	%f	]	Gyro:	[	%f	%f	%f	]	Mag: [	%f	%f	%f	]", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
			
	}
