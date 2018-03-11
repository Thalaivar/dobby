#include "dobby.h"
#include <iostream>

using namespace std;

Dobby dobby;

int main(){

  // program has started
  dobby.state = RUN;

  dobby.setup();

  dobby.pre_flight_checks();

  cout << dobby.state << endl;

  while(true){
  	
	dobby.imu.print_tb_angles();
  }
  return 0;
}
