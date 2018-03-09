#include <pthread.h>
#include <iostream>
#include <cstdlib>
//This can be the main thread that is run when testing:
//this main 
//calibarates the imu, sets esc, and updates flight mode
//waits for arming:
//then runs controller, and arming check in background
// if controller exits with error, then esc values are set to 1000 and mode is set to default and arming state is false
// if controller exits safely ? figid it out:

using namespace std;

void beat(){
	cout << "beating heart...." <<endl;
}
void *heartbeat(void *threadid){
	//write heartbeat code here
	
	for(int i =0; i <10 ; i++){
		beat();
	}
	pthread_exit(NULL);
}
int main(){
	pthread_t led_blink;
	pthread_attr_t attr;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	void *ls;
	int status;
	int id = 0;
	status = pthread_create(&led_blink, NULL, heartbeat, (void *) id);
	if (status ){
		cout << "Heart-beat Initiation Failed" << endl;
		exit(-1);
	}
	else
		cout << "Heartbeating..." << endl;
	status = pthread_join(led_blink, &ls);
	if(status){
		cout << "UNable to join thread" << endl;
		exit(-1);
	}
	else{
		cout << "exiting main thread" << endl;
	}
	pthread_exit(NULL);
}

