#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>

#include "interpolate.hpp"
#include "resampling.hpp"
#include "detect_line.hpp"
#include "navigation.hpp"


#include "camera_parameters.h"
using namespace std;

#define STEER_P 0.25
#define SPEED_DEC 0.5

#define tic      double tic_t = clock();
#define toc      std::cout << (clock() - tic_t)/CLOCKS_PER_SEC \
                           << " seconds" << std::endl;

//Needds to be computed from camera calibration results using resampling.c

double cam_ct[12];
double bot_pos_in_cam[4];
double bot_pos_in_world[4];

char line_detection_kernel[9] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };

#define NB_LINES_SAMPLED 16
#define SAMPLE_SPACING_MM 20.0

float posx_samples_world[NB_LINES_SAMPLED];
unsigned int posv_samples_cam[NB_LINES_SAMPLED];

float rand_a_b(int a, int b) {
	//return ((rand() % (b - a) + a;
	float rand_0_1 = (((float) rand()) / ((float) RAND_MAX));
	return (rand_0_1 * (b - a)) + a;
}

void kernel_horiz(Mat & img, int * kernel_response, unsigned int v) {
	int i;
	kernel_response[0] = 0;
	unsigned char * upixel_ptr = (unsigned char *) img.data;
	for (i = 1; i < (img.cols - 1); i++) {
		int response = 0;
		response -= upixel_ptr[(v - 1) * img.step + (i - 1)];
		response -= upixel_ptr[(v) * img.step + (i - 1)];
		response -= upixel_ptr[(v + 1) * img.step + (i - 1)];

		response += upixel_ptr[(v - 1) * img.step + (i + 1)];
		response += upixel_ptr[(v) * img.step + (i + 1)];
		response += upixel_ptr[(v + 1) * img.step + (i + 1)];

		kernel_response[i] = response;

	}
	kernel_response[i] = 0;
}

float distance_to_curve(curve * l, float x, float y) {
	float resp = 0.;
	int j;
	for (j = 0; j < POLY_LENGTH; j++) {
		resp += l->p[j] * pow(x, j);
	}
	float error = abs(resp - y);
	return error;
}

#define RANSAC_LIST (POLY_LENGTH)
#define RANSAC_NB_LOOPS 10
#define RANSAC_INLIER_LIMIT 2.0
float fit_line(point * pts, unsigned int nb_pts, curve * l) {
	int i;
	float * x = (float *) malloc(nb_pts * sizeof(float));
	float * y = (float *) malloc(nb_pts * sizeof(float));
	char * used = (char *) malloc(nb_pts * sizeof(char));
	unsigned char * inliers = (unsigned char *) malloc(nb_pts * sizeof(char));
	unsigned char * max_inliers = (unsigned char *) malloc(
			nb_pts * sizeof(char));
	int max_consensus = 0;
	for (i = 0; i < RANSAC_NB_LOOPS; i++) {
		int pt_index = 0;
		int nb_consensus = 0;
		unsigned int idx = 0;
		float max_x_temp = 0;
		float min_x_temp = 3000.; //arbitrary ...
		memset(used, 0, nb_pts * sizeof(char)); //zero used index
		while (pt_index < RANSAC_LIST) {
			idx = rand_a_b(0, (nb_pts - 1));
			while (used[idx] != 0)
				idx = (idx + 1) % NB_LINES_SAMPLED;
			y[pt_index] = pts[idx].y;
			x[pt_index] = pts[idx].x;
			if (x[pt_index] > max_x_temp)
				max_x_temp = x[pt_index];
			used[idx] = 1;
			inliers[nb_consensus] = idx;
			nb_consensus++;
			pt_index++;
		}
		//From initial set, compute polynom
		compute_interpolation(x, y, l->p, POLY_LENGTH, pt_index);
		max_x_temp = 0;
		idx = 0;
		for (idx = 0; idx < nb_pts; idx++) {
			if (used[idx] == 1) {
				continue;
			}
			used[idx] = 1;

			//Distance should be computed properly
			float error = distance_to_curve(l, pts[idx].x, pts[idx].y);
			if (error < RANSAC_INLIER_LIMIT) {
				inliers[nb_consensus] = idx;
				nb_consensus++;
				if (pts[idx].x > max_x_temp)
					max_x_temp = pts[idx].x;
				if (pts[idx].x < min_x_temp)
					min_x_temp = pts[idx].x;
			}
			if (nb_consensus > max_consensus) {
				max_consensus = nb_consensus;
				l->max_x = max_x_temp;
				l->min_x = min_x_temp;
				memcpy(max_inliers, inliers, nb_consensus * sizeof(char));
			}
		}
	}

	for (i = 0; i < max_consensus; i++) {
		int idx = max_inliers[i];
		y[i] = pts[idx].y;
		x[i] = pts[idx].x;
	}
	//Should recompute the curve interpolation with inliers
	compute_interpolation(x, y, l->p, POLY_LENGTH, max_consensus);
	free(x);
	free(y);
	free(used);
	free(inliers);
	free(max_inliers);
	//printf("Max consensus %d \n", max_consensus);
	/*
	 printf("%f + %f*x + %f*x^2 \n", l->p[0], l->p[1], l->p[2]);
	 */
	float confidence = ((float) (max_consensus))
			/ ((float) nb_pts);
	//printf("Confidence %f \n", confidence);

	return confidence;
}

#define SCORE_THRESHOLD 500
#define WIDTH_THRESHOLD 100
float detect_line(Mat & img, curve * l, point * pts, int * nb_pts) {
	int i, j;
	(*nb_pts) = 0;
	int * sampled_lines = (int *) malloc(img.cols * sizeof(int));
	srand(time(NULL));
	for (i = 0; i < NB_LINES_SAMPLED; i++) {
		kernel_horiz(img, sampled_lines, posv_samples_cam[i]);
		int max = 0, min = 0, max_index = 0, min_index = 0, sig = 0;
		int score, width;
		for (j = 1; j < (img.cols - 1); j++) {
			//Track is white on black, so we expect maximum gradient then minimum
			if (sampled_lines[j] < min) {
				if (max > 0) { //we have a signature ...
					min = sampled_lines[j];
					min_index = j;
					sig = 1;
				}
			}
			if (sampled_lines[j] > max) {
				if (min > 0) {
					min = 0;
					max = sampled_lines[j];
					max_index = j;
					sig = 0;
				} else {
					max = sampled_lines[j];
					max_index = j;
				}
			}
		}
		score = abs(min) + abs(max);
		width = max_index - min_index;
		if (sig == 1 && score > SCORE_THRESHOLD && width < WIDTH_THRESHOLD) {
			pts[(*nb_pts)].x = ((float) (max_index + min_index)) / 2.;
			pts[(*nb_pts)].y = posv_samples_cam[i];
			(*nb_pts)++;
		}
	}
	free(sampled_lines);
	for (i = 0; i < (*nb_pts); i++) {
		//undistort_radial(K, pts[i].x, pts[i].y, &(pts[i].x), &(pts[i].y),radial_distort, 2);
		pixel_to_ground_plane(cam_ct, pts[i].x, pts[i].y, &(pts[i].x),
				&(pts[i].y));
	}

	if ((*nb_pts) > (POLY_LENGTH * 2)) {
		return fit_line(pts, (*nb_pts), l);
	} else {
		return 0.;
	}
//return 0.;
//TODO : for each detected point, compute its projection in the world frame instead of plain image coordinates
//

}

void init_line_detector() {
	int i;
	float x, y, u, v;
	calc_ct(camera_pose, K, cam_to_bot_in_world, cam_ct); //compute projection matrix from camera coordinates to world coordinates
	//Sampling world frame and projecting into camera frame
	for (i = 0; i < NB_LINES_SAMPLED; i++) {
		posx_samples_world[i] = (i + 1) * SAMPLE_SPACING_MM;
		ground_plane_to_pixel(cam_ct, posx_samples_world[i], 0, &u, &v);
		posv_samples_cam[i] = v;
	}
}

int detect_line_test(int argc, char ** argv) {
	int i, nb_pts;
	float x, y, u, v, y_lookahead, speed_factor, curvature;
	curve detected;
	point pts[NB_LINES_SAMPLED];
	if (argc < 2) {
		printf("Requires image path \n");
		exit(-1);
	}

	init_line_detector();
	Mat line_image;
	line_image = imread(argv[1], IMREAD_GRAYSCALE);
	Mat map_image(320, 320,
	CV_8UC1, Scalar(255));
	tic
	;
	detect_line(line_image, &detected, pts, &nb_pts);
	toc
	;
	for (i = 0; i < nb_pts; i++) {
		ground_plane_to_pixel(cam_ct, (double) pts[i].x, (double) pts[i].y, &u,
				&v);
		circle(line_image, Point((int) u, (int) v), 4, Scalar(0, 0, 0, 0), 4, 8,
				0);

		circle(map_image, Point(pts[i].x, (pts[i].y + map_image.cols / 2)), 1,
				Scalar(0, 0, 0, 0), 4, 8, 0);
	}

	for (x = detected.min_x;; x += 0.1) {
		float resp = 0.;
		for (i = 0; i < POLY_LENGTH; i++) {
			resp += detected.p[i] * pow(x, i);
		}
		ground_plane_to_pixel(cam_ct, (double) x, (double) resp, &u, &v);
		if (u > line_image.cols || u < 0 || v > line_image.rows || v < 0)
			break;
		circle(line_image, Point((int) u, (int) v), 1, Scalar(0, 0, 0, 0), 2, 8,
				0);
	}
	curvature = steering_speed_from_curve(&detected, 150.0,
							&y_lookahead , &speed_factor);
	cout << "Curvature " << curvature << endl ;
	cout << "Y lookahead " << y_lookahead << endl ;
	imshow("orig", line_image);
	imshow("map", map_image);
	waitKey(0);
	return 0;
}
