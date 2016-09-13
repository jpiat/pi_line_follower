#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace Eigen ;
#ifndef RESAMPLING_H
#define RESAMPLING_H

void distort_radial(double * K, float u, float v, float * ud,
		float * vd, double * poly, unsigned int poly_size);

void undistort_radial(double * K, float u, float v, float * ud,
		float * vd, double * poly, unsigned int poly_size);

void homography(char * image_data, int w, int h, char * img_h, double * H,
		double * K1, double * K2);

void calc_ct(double * world_to_cam, double * K, double * world_to_bot, double *Ct);
void pixel_to_ground_plane(double * Ct, float u, float v, float * x,
		float * y);
void ground_plane_to_pixel(double * Ct, double x, double y, float * u,
		float * v);
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

#endif
