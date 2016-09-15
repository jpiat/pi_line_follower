#include "servo_control.h"

#ifdef __arm__
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
#else
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
#ifdef __arm__
	if (gpioInitialise() < 0) {
		printf("Cannot initialise GPIO \n");
		exit(-1);
		// pigpio initialisation failed.
	}
#endif
	printf("Arming ESC \n");
	arm_esc();
	for (i = -1.0; i < 1.0; i += 0.1) {
		set_servo_angle(i);
	}
	gpioTerminate();
	return 1;
}
#endif
