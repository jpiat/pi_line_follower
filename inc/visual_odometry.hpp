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
#define DESCRIPTOR_LENGTH 256 //Need to test different length and threshold
#define DESCRIPTOR_MATCH_THRESHOLD 48
#define STACK_SIZE 100

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

typedef struct fxy {
	float x;
	float y;
} fxy;

void init_visual_odometry();
int estimate_ground_speeds(Mat & img,fxy * speeds);

int test_estimate_ground_speeds(int argc, char ** argv);
#endif
