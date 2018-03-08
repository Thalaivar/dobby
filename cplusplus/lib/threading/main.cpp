#include <pthread.h>
//This can be the main thread that is run when testing:
//this main : calibarates the imu, sets esc, and updates flight mode
// Setting just for blinking led right now

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

