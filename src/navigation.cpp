#include <iostream>
#include <unistd.h>
#include <math.h>
#include "detect_line.hpp"


float steering_speed_from_curve(curve * c, float x_lookahead, float * y_lookahead, float * speed) {
	(*y_lookahead) = 0;
	(*speed) = 1.0 ;
	if(x_lookahead > c->max_x || x_lookahead < c->min_x){
		(*speed) = c->max_x/x_lookahead ;
		x_lookahead = c->max_x ;//may not be the best idea ...

	}
	int i;
	for (i = 0; i < POLY_LENGTH; i++) {
		(*y_lookahead) += c->p[i] * pow(x_lookahead, i);
	}
	float D_square = pow(x_lookahead, 2) + pow((*y_lookahead), 2);
	float r = D_square / (2.0 * (*y_lookahead));
	float curvature = 1000.0 / r; //to have in milimeters instead of meters
	return curvature;
}
