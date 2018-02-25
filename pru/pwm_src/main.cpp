#include "pwm.h"

Motors motors;

int main(void){
			scanf("%d %d %d %d", &motors.channel_val[0], &motors.channel_val[1], &motors.channel_val[2], &motors.channel_val[3]);
			motors.update();
}
