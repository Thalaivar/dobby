#include "pwm.h"

int main(void){
	initialize_pru();
	write_pwm_pulse_us(1, 2000);
	while( true ){}
	disable_pru();

	return 0;
}
