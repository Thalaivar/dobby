#include <pthread.h>
//This can be the main thread that is run when testing:
//this main 
//calibarates the imu, sets esc, and updates flight mode
//waits for arming:
//then runs controller, and arming check in background
// if controller exits with error, then esc values are set to 1000 and mode is set to default and arming state is false
// if controller exits safely ? figid it out:


void *heartbeat(){
	//write heartbeat code here
	
	pthread_exit();
}
int main(){
	pthread_t led_blink;
	int status;

	status = pthread_create(&led_blink, NULL, heartbeat, NULL);
	if (status)
		cout << "Heart-beat Initiation Failed" << endl;
	else
		cout << "Heartbeating..." << endl;

}

