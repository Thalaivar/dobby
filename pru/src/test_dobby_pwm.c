#include "dobby_pwm.h"

int write_to_motors();

void main(){

    init_pwm_pru();
	write_to_motors();
	exec_dobby_pwm();

	printf("STARTING\n");

    while(1){
			int choose = write_to_motors();
			if(choose == -1) break;
        }
	  printf("DONE!");
	  stop_pwm_pru();
}

int write_to_motors() {
		
    	  int option = 0;	 
	      unsigned int channel_width[4];
	   	  printf("Enter 1 to write pwm, -1 to exit: ");
		  scanf("%d\n", &option);
          if(option){
		      printf("Enter PWM for C1 C2 C3 C4: ")
              scanf("%d %d %d %d\n", &channel_width);
              write_pwm_pulsewidth(1, channel_width[0]);
              write_pwm_pulsewidth(2, channel_width[1]);
              write_pwm_pulsewidth(3, channel_width[2]);
              write_pwm_pulsewidth(4, channel_width[3]);
		   }

          return option;


	}
