#include "ppm.h"
#include <stdio.h>
#include <chrono>
#include <thread>

#define NUM_LOOPS 120000

Receiver recv;

int main(){
  if(recv.init_radio()<0) cout << "FAIL!" << endl;
  if(recv.is_initialized){
   for(int i = 0; i < NUM_LOOPS; i++){
      recv.update();
	  printf("%d | %d | %d | %d | %d | %d |\n", recv.recv_channel[0], recv.recv_channel[1], recv.recv_channel[2], \
                                               recv.recv_channel[3], recv.recv_channel[4], recv.recv_channel[5]);
    }
	std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
  return 0;
}
