#include "opencv2/highgui/highgui_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc_c.h"


#ifndef DETECT_LINE_H
#define DETECT_LINE_H
#define POLY_LENGTH 3
typedef struct curve {
	float p[POLY_LENGTH];
	float max_x;
	float min_x;
} curve;

typedef struct point {
	float x;
	float y;
} point;


float detect_line(IplImage * img, curve * l, point * pts, int * nb_pts);
int detect_line_test(int argc, char ** argv) ;
#endif
