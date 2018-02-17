#include "dobby_pwm.h"

void main(){

      init_pwm_pru();
	  exec_dobby_pwm();
	  
	  printf("STARTING\n");
	  int i = 0;
	  for(i = 0; i < 100; i++){
	  	for(int j = 1; j < 5; j++){
			write_pwm_pulsewidth(j, 1000+i);	
		}
		printf("%d\n", i);
	}
	  printf("DONE!");
	  stop_pwm_pru();
}
