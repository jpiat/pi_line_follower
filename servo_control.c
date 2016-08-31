#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

#include <pigpio.h>

#define SERVO_1 12
#define SERVO_2 13

int main(void){
	int i  ;
	if (gpioInitialise() < 0)
	{
  	printf("Cannot initialise GPIO \n");
	exit(-1);
	 // pigpio initialisation failed.
	}
	for(i = 500 ; i < 2500 ; i ++){
		gpioServo(SERVO_1, i);
		sleep(100);
	}
	gpioTerminate();
	return 1 ;
}
