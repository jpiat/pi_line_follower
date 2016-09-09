//all the following is computed with matlab calibration toolbox ...

double radial_distort = { -0.3022,    0.0969}; // need to inverse functions to compute undistort coordinates
double K[9] = { 437.4347, 0, 0, 0, 437.3542, 0, 279.5110, 219.7862, 1.0000 };
double camera_pose = { -0.0356, -0.5803, 0.8136, -78.2420, 0.9993, -0.0089,
		0.0374, 70.3447, -0.0145, 0.8144, 0.5802, 171.7371 };
