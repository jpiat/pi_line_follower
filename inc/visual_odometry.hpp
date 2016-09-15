#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

#include "resampling.hpp"
#include "camera_parameters.h"

extern "C" {
#include "fast/fast.h"
}

using namespace cv;
using namespace std;

#define DESCRIPTOR_WINDOW 32
#define DESCRIPTOR_LENGTH 256
#define DESCRIPTOR_MATCH_THRESHOLD 64
#define STACK_SIZE 100

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H
int estimate_ground_speeds(Mat & img, unsigned int start_line, double * Ct,
		float * speeds);

int test_estimate_ground_speeds(int argc , char ** argv);
#endif
