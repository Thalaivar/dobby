/**************************************************
                imu library
***************************************************/

#ifndef IMU_H
#define IMU_H

#include <roboticscape.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>	// for uint8_t types etc
#include <sys/stat.h>
#include <time.h>		// usleep, nanosleep
#include <math.h>		// atan2 and fabs
#include <signal.h>		// capture ctrl-c
#include <pthread.h>	// multi-threading
#include <linux/input.h>// buttons
#include <poll.h> 		// interrupt events
#include <sys/mman.h>	// mmap for accessing eQep
#include <sys/socket.h>	// udp socket
#include <netinet/in.h>	// udp socket
#include <sys/time.h>
#include <arpa/inet.h>	// udp socket
#include <ctype.h>		// for isprint()
#include <sys/select.h>	// for read timeout

#define RAD_TO_DEG		57.295779513

#define TB_PITCH_X	0
#define TB_ROLL_Y	1
#define TB_YAW_Z	2

class IMU{
  public:
    rc_imu_config_t config;
    rc_imu_data_t   data;

    IMU();
    int init_imu();
    void print_tb_angles();
    bool is_initialized;
};
#endif
