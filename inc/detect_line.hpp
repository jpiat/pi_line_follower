#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace Eigen;
using namespace cv;


#ifndef DETECT_LINE_H
#define DETECT_LINE_H
#define POLY_LENGTH 4
typedef struct curve {
	float p[POLY_LENGTH];
	float max_x;
	float min_x;
} curve;

typedef struct point {
	float x;
	float y;
} point;


float detect_line(Mat & img, curve * l, point * pts, int * nb_pts);
void init_line_detector() ;
int detect_line_test(int argc, char ** argv) ;
#endif
