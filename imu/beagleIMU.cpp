#include "beagleIMU.h"


void beagle_imu::set_configuration_default(rc_imu_config_t* config){

		rc_set_imu_config_to_defaults(config);
	}

void beagle_imu::update_random(rc_imu_data_t* data){

		// some checks in here??
		rc_read_accel_data(data);
		rc_read_gyro_data(data);
		rc_read_mag_data(data);
	}

void beagle_imu::init_random(rc_imu_data_t* data, rc_imu_config_t config){

		//check if init was successful
		printf("Initializing IMU for random mode operation....\n");
		rc_initialize_imu(data, config);
		printf("IMU initialized for random mode!\n");

	}

void beagle_imu::enable_mag(rc_imu_config_t* config){

		config->enable_magnetometer = 1;

	}

void beagle_imu::print_config(rc_imu_config_t* config){

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

void beagle_imu::set_operation_mode(imu_mode* mode, imu_mode choice){

				//check if mode chosen is appropriate
				if(choice == RANDOM_MODE || choice == DMP_MODE){
							*mode = choice;
					}

				else printf("Invalid imu mode chosen!\n");

	}

void beagle_imu::debug_print_vals(rc_imu_data_t* data){

			printf("Accel: [	%f	%f	%f	]	Gyro:	[	%f	%f	%f	]	Mag: [	%f	%f	%f	]", data->accel[0], data->accel[1], data->accel[2], data->gyro[0], data->gyro[1], data->gyro[2], data->mag[0], data->mag[1], data->mag[2]);

	}
