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
	      unsigned int channel_width_1, channel_width_2, channel_width_3, channel_width_4;
	   	  printf("Enter 1 to write pwm, -1 to exit: ");
		  scanf("%d", &option);
          if(option == 1){
		      printf("Enter PWM for C1 C2 C3 C4: ");
              scanf("%d %d %d %d", &channel_width_1, &channel_width_2, &channel_width_3, &channel_width_4);
              write_pwm_pulsewidth(1, channel_width_1);
              write_pwm_pulsewidth(2, channel_width_2);
              write_pwm_pulsewidth(3, channel_width_3);
              write_pwm_pulsewidth(4, channel_width_4);
		   }

          return option;


	}
