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

void arm_esc() {
	gpioServo(ESC, ARM_ESC);
	sleep(2);
}

void set_esc_speed(float speed) {
	float cmd = CENTER_ESC + ((MAX_ESC - CENTER_ESC) * speed);
	gpioServo(ESC, (unsigned int) cmd);
}

void set_servo_angle(float angle) {
	float cmd = CENTER_SERVO + ((MAX_SERVO - CENTER_SERVO) * angle);
	gpioServo(ESC, (unsigned int) cmd);
}

int test_servo(void) {
	float i;
	if (gpioInitialise() < 0) {
		printf("Cannot initialise GPIO \n");
		exit(-1);
		// pigpio initialisation failed.
	}
	printf("Arming ESC \n");
	arm_esc();
	for (i = -1.0; i < 1.0; i += 0.1) {
		set_servo_angle(i);
	}
	gpioTerminate();
	return 1;
}
