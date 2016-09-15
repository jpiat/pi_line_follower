#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

#ifdef __arm__
#include <pigpio.h>
#endif


#define ESC 12
#define SERVO 13

#define MIN_ESC 800
#define CENTER_ESC 1500
#define MAX_ESC  1600
#define ARM_ESC 1500

#define MIN_SERVO 1000
#define CENTER_SERVO 1500
#define MAX_SERVO 2000

#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

void arm_esc();
void set_esc_speed(float speed);
void set_servo_angle(float angle);
int test_servo(void);
#endif
