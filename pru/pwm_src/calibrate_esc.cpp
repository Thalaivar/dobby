#include "pwm.h"

Motors motors = Motors();

int main(){
    if(motors.calibrate_esc() == 0) printf("ESC Calibrated!\n");
    motors.disable_pru();
    return 0;
}
