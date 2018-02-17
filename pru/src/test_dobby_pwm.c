#include "dobby_pwm.h"

void main(){

    init_pwm_pru();
	  exec_dobby_pwm();

	  printf("STARTING\n");
    int option = 0;
    unsigned int channel_width[4];

    while(1){

          scanf("Enter 1 to set PWM, -1 to exit: %d", &option);
          if(option){
              scanf("Enter PWM for C1: %d ; C2: %d ; C3: %d ; C4: %d", &channel_width);
              write_pwm_pulsewidth(1, channel_width[0]);
              write_pwm_pulsewidth(2, channel_width[1]);
              write_pwm_pulsewidth(3, channel_width[2]);
              write_pwm_pulsewidth(4, channel_width[3]);
              stop_pwm_pru();
           }

          else  break;
      }
	  printf("DONE!");
	  stop_pwm_pru();
}
