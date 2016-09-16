#include "servo_control.h"

#ifdef __arm__

int init_servo() {
	return gpioInitialise();
}
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

int close_servo() {
	gpioTerminate();
	return 1;
}
#else

int init_servo() {
	printf("Arming servo system \n");
	return 1;
}

int close_servo() {
	printf("Closing servo system \n");
	return 1;
}

void arm_esc() {
	printf("Arming esc \n");
}

void set_esc_speed(float speed) {
	float cmd = CENTER_ESC + ((MAX_ESC - CENTER_ESC) * speed);
	printf("Setting esc to %f \n", cmd);
}

void set_servo_angle(float angle) {
	float cmd = CENTER_SERVO + ((MAX_SERVO - CENTER_SERVO) * angle);
	printf("Setting servo to %f \n", cmd);
}

int test_servo(void) {
	float i;
	init_servo();
	printf("Arming ESC \n");
	arm_esc();
	for (i = -1.0; i < 1.0; i += 0.1) {
		set_servo_angle(i);
	}
	close_servo();
	return 1;
}
#endif
