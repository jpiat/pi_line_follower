#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define FPS 60
#define HFLIP 0
#define VFLIP 0


//all the following is computed with matlab calibration toolbox ...
extern double radial_distort[2];
#define POLY_UNDISTORT_SIZE 3
extern double radial_undistort[POLY_UNDISTORT_SIZE];

//Eigen expect buffer to be constructed in column first
extern double K[9];
//pose is extracted from Matlab and then shifted to have centered Y
extern double camera_pose[12] ;
//translate to bot position in ground plane
//bot frame of reference is oriented with x axis pointing forward and y axis pointing right from the robot
//perspective.
extern double cam_to_bot_in_world[16] ;

extern double cam_ct[12];
