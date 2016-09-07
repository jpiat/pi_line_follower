#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

#include <pigpio.h>

#define ESC 12
#define SERVO 13

#define MIN_ESC 800
#define CENTER_ESC 1500
#define MAX_ESC  1600
#define ARM_ESC 1500

#define MIN_SERVO 1000
#define CENTER_SERVO 1500
#define MAX_SERVO 2000

int test_servo(void){
	int i  ;
	if (gpioInitialise() < 0)
	{
  	printf("Cannot initialise GPIO \n");
	exit(-1);
	 // pigpio initialisation failed.
	}
	gpioServo(ESC, ARM_ESC);
	printf("Arming ESC \n");
	sleep(2);
	for(i = MIN_SERVO ; i < MAX_SERVO ; i +=10){
		gpioServo(SERVO, i);
		usleep(50000);
	}
	/*
	for(i = MIN_ESC ; i < MAX_ESC ; i +=10){
                gpioServo(ESC, i);
                usleep(50000);
        }*/

	gpioTerminate();
	return 1 ;
}
