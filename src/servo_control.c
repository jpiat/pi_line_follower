#include "servo_control.h"

#ifdef __arm__

int init_servo() {
	return gpioInitialise();
}

void arm_esc() {
	gpioServo(ESC, ARM_ESC_1);
	sleep(1);
	gpioServo(ESC, ARM_ESC_2);
	sleep(2);
}

void set_esc_speed(float speed) {
	if(speed > 1.0) speed = 1.0 ;
	if(speed < -1.0) speed = -1.0;
	float cmd = CENTER_ESC + ((MAX_ESC - CENTER_ESC) * speed);
	gpioServo(ESC, (unsigned int) cmd);
}

void set_servo_angle(float angle) {
	if(angle > 1.0) angle = 1.0;
	if(angle < -1.0) angle = -1.0;
	float cmd = CENTER_SERVO + ((MAX_SERVO - CENTER_SERVO) * angle);
	gpioServo(SERVO, (unsigned int) cmd);
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
#endif

int test_servo(void) {
	int i;
	init_servo();
	printf("Arming ESC \n");
	arm_esc();
	printf("ESC armed and ready to go \n");
	for(i = 0 ; i < 1024; i ++) {
		float angle = -1.0 + i*(2./1024.0);
		//printf("%f \n", angle);
		set_servo_angle(angle);
		usleep(10000);
	}
	close_servo();
	return 1;
}
