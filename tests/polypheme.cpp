#include "camera_parameters.h"
#include <iostream>
#include <unistd.h>
#include <math.h>
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/core/core.hpp"

#include "detect_line.hpp"
#include "visual_odometry.hpp"
#include "navigation.hpp"

extern "C" {
#include "servo_control.h"
}

using namespace std;
using namespace cv;

#define POLE_INPUT 17
#define DISTANCE_TO_TRAVEL 130000.0
#ifdef PI_CAM
#include "RaspiCamCV.h"

RaspiCamCvCapture * capture;
VideoCapture capture_from_file;
int input_is_file = 0;

int initCaptureFromCam() {
	RASPIVID_CONFIG * config = (RASPIVID_CONFIG*)malloc(sizeof(RASPIVID_CONFIG));
	RASPIVID_PROPERTIES * properties = (RASPIVID_PROPERTIES*)malloc(sizeof(RASPIVID_PROPERTIES));
	config->width=IMAGE_WIDTH;
	config->height=IMAGE_HEIGHT;
	config->bitrate=0;      // zero: leave as default
	config->framerate=FPS;
	config->monochrome=1;
	properties->hflip = HFLIP;
	properties->vflip = VFLIP;
	properties -> sharpness = 0;
	properties -> contrast = 0;
	properties -> brightness = 50;
	properties -> saturation = 0;
	properties -> exposure = AUTO;
	properties -> shutter_speed = 0;// 0 is autoo
	capture = (RaspiCamCvCapture *) raspiCamCvCreateCameraCapture3(0, config, properties, 1);
	return 1;
}
int initCaptureFromFile(char * path) {
	if (!capture_from_file.open(path)) {
		cout << "Capture from "<<path<<" didn't work" << endl;
		return 0;
	}
	input_is_file = 1;
	return 1;
}

Mat getFrame() {
	if(input_is_file) {
		Mat img;
		capture_from_file >> img;
		return img;
	} else {
		int success;
		do {
			success = raspiCamCvGrab(capture);
			if(success == 0) usleep(500);
		}while(success == 0);
		IplImage* image = raspiCamCvRetrieve(capture);
		Mat img = cvarrToMat(image);
		/*Mat img(image->height, image->width, CV_8UC1, Scalar(0));
		 for(i = 0 ; i < image->height ; i ++){
		 memcpy(&(img.data[i*img.step]), &(image->imageData[i*image->widthStep]), image->width*image->nChannels);
		 }*/
		return img;
	}}

#else

VideoCapture capture;

int initCaptureFromCam() {
	if (!capture.open(0)) {
		cout << "Capture from camera #0 didn't work" << endl;
		return 0;
	}
	return 1;
}
int initCaptureFromFile(char * path) {
	if (!capture.open(path)) {
		cout << "Capture from " << path << " didn't work" << endl;
		return 0;
	}
	return 1;
}

Mat getFrame() {
	Mat img;
	capture >> img;
	return img;
}

#endif

#define STEER_P -0.20
#define SPEED_DEC 0.5
int main(void) {
	int update = 0;
	int alive = 0;
	int frame_counter = -1;
	int old_state = 1;
	int i;
	int rising_edge = 0, falling_edge = 0;
	int detect_line_timeout = 10;
	int nb_points;
	double travelled_distance = 0.;
	Mat map_image(320, 320,
	CV_8UC1, Scalar(255));
	point pts[16];
	curve line;
	fxy speed;
	float y_lookahead;
	init_line_detector();
	init_visual_odometry();
#ifdef DEBUG
	if(argc > 1) {
		cout << "Need a path to video in testing" << endl;
		exit(-1);
	}
	initCaptureFromFile(argv[1]);
#else
	initCaptureFromCam();
#endif

	init_servo();
#ifdef __arm__
	gpioSetMode(POLE_INPUT, PI_INPUT);
	gpioSetPullUpDown(POLE_INPUT, PI_PUD_UP);
#endif
	arm_esc();
	set_servo_angle(0.0);
	//alive = 1; //to be removed when not debugging
	while (1) {
		Mat img = getFrame();
		if (alive == 1) {
			if (frame_counter > 0) {
				frame_counter--;
				continue;
			} else {
				float confidence = detect_line(img, &line, pts, &nb_points);
				//cout << "Confidence " << confidence << endl;
				if (confidence < 0.35) {
					//should we consider updating the command when we have a low confidence in the curve estimate
					detect_line_timeout--;
					update = 0;
				} else {
					detect_line_timeout = 10;
					update = 1;
				}
				//cout << "line detector used " << nb_points << endl;
				if (estimate_ground_speeds(img, &speed)) {
					//cout << "speed " << speed.x << ", " << speed.y << endl;
					travelled_distance += sqrt(
							pow(speed.x, 2) + pow(speed.y, 2));
					//TODO: use a kind of PID for the ESC control or map the robot position using integral of speed over time
				}
				//imshow("img", img);
				//waitKey(0);
				if (update == 1) {
					float speed_factor;
					float steering = steering_speed_from_curve(&line, 150.0,
							&y_lookahead, &speed_factor); //lookahead point should evolve with speed
					float angle_from_steering = steering * STEER_P;
					float speed_from_steering = 1.0
							- (abs(angle_from_steering) * SPEED_DEC);
					speed_from_steering *= speed_factor;
					//TODO: apply command to servo and esc
					/*cout << "speed factor:" << speed_factor << endl ;
					cout << "speed :" << speed_from_steering << endl ;
					cout << "steering :" << angle_from_steering << endl ;*/
					//set_esc_speed(speed_from_steering);
					set_servo_angle(angle_from_steering);
				}
				//TODO:detect falling edge on IO or no line was seen for more than 10 frames
				if (travelled_distance >= DISTANCE_TO_TRAVEL
						|| detect_line_timeout <= 0) {
					cout << "distance travelled " << endl;
					alive = 0;
					//set_esc_speed(0.);
					set_servo_angle(0.);
					close_servo();
					sleep(2);
					exit(0);
				}
			}
		} else {
			if (rising_edge) {
				alive = 1;
				cout << "countdown to start " << endl;
				frame_counter = 60; //initialize a 2sec timeout before robot starts
				travelled_distance = 0.;
			}
		}
#ifdef __arm__
		//cout << gpioRead(POLE_INPUT) << endl ;
		rising_edge = (old_state == 0) & (gpioRead(POLE_INPUT) == 1);
		falling_edge = (old_state == 1) & (gpioRead(POLE_INPUT) == 0);
		old_state = gpioRead(POLE_INPUT);
#else
		rising_edge = 1;
		falling_edge = 0;
#endif
	}

}
