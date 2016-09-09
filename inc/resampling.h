#include <stdio.h>

#include "mat_types.h"
#include "double_mat_ops.h"

double distort_plumb_bob(double xn, double yn, double * xd, double * yd,
		double * poly);

void homography(char * image_data, int w, int h, char * img_h, double * H,
		double * K1, double * K2);

void calc_ct_and_H(double * world_to_cam, double * K, double *Ct, double * H);
void pixel_to_ground_plane(double * Ct, double u, double v, float * x,
		float * y);

/**
 * Poses are expressed in world frame whose origin is center of the scene on the ground plane
 * n is plane normal in world frame, d is plane distance to origin in world frame
 */
void compute_homography_from_cam_cam_pos(double * C1_pose, double * C2_pose,
		double * n, double d, double * K1, double * K2, double *H);

int compute_ground_pixel_projection(double * uv_c1, double * uv_c2, double * n,
		double d, double * C1_matrix, double * C2_matrix);

void map_from_homography(char * img_in, unsigned int w_in, unsigned int h_in,
		char * img_out, unsigned int w_out, unsigned int h_out, double * H,
		double scale);

