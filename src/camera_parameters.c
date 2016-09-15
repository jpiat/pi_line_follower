//all the following is computed with matlab calibration toolbox ...
double radial_distort[2] = { -0.3070, 0.1031 }; // need to inverse functions to compute undistort coordinates

//Eigen expect buffer to be constructed in column first
double K[9] = { 443.4681, 0, 0, 0, 442.9819, 0, 326.2723, 233.9095, 1.0000 };
//pose is extracted from Matlab and then shifted to have centered Y
double camera_pose[12] = { 0.1530, 0.5648, -0.8109, -0.9879, 0.1077, -0.1114,
		0.0244, 0.8181, 0.5745, 71.8637, -99.2682, 405.3535 };

#define TX_WORLD_TO_BOT 298.997
#define TY_WORLD_TO_BOT 121.19
//translate to bot position in ground plane
//bot frame of reference is oriented with x axis pointing forward and y axis pointing right from the robot
//perspective.
double cam_to_bot_in_world[16] = { -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0,
TX_WORLD_TO_BOT, TY_WORLD_TO_BOT, 0, 1 };

double cam_ct[12];
