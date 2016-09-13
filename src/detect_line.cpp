#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <iostream>

#include "interpolate.hpp"
#include "resampling.hpp"
#include "detect_line.hpp"

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

#define IMG_HEIGHT 480
#define SAMPLES_FIRST_LINE 8
#define SAMPLES_LAST_LINE IMG_HEIGHT
#define NB_LINES_SAMPLED 16
#define SAMPLES_SPACE ((SAMPLES_LAST_LINE - SAMPLES_FIRST_LINE)/NB_LINES_SAMPLED)

float rand_a_b(int a, int b) {
	//return ((rand() % (b - a) + a;
	float rand_0_1 = (((float) rand()) / ((float) RAND_MAX));
	return (rand_0_1 * (b - a)) + a;
}

void kernel_horiz(Mat & img, int * kernel_response, int v) {
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
	curve fct;
	int max_consensus = 0;
	for (i = 0; i < RANSAC_NB_LOOPS; i++) {
		int pt_index = 0;
		int nb_consensus = 0;
		unsigned int idx = 0;
		float max_x_temp = 0;
		float min_x_temp = 3000.; //arbitrary ...
		memset(used, 0, NB_LINES_SAMPLED * sizeof(char)); //zero used index
		while (pt_index < RANSAC_LIST) {
			//Select set of samples, with distance constraint
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
		compute_interpolation(x, y, fct.p, POLY_LENGTH, pt_index);
		max_x_temp = 0;
		idx = 0;
		for (idx = 0; idx < nb_pts; idx++) {
			if (used[idx] == 1) {
				continue;
			}
			used[idx] = 1;

			//Distance should be computed properly
			float error = distance_to_curve(&fct, pts[idx].x, pts[idx].y);
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
				//memcpy(l->p, fct_params, POLY_LENGTH * sizeof(float));
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
	printf("Max consensus %d \n", max_consensus);
	/*
	 printf("%f + %f*x + %f*x^2 \n", l->p[0], l->p[1], l->p[2]);
	 */
	float confidence = ((float) (max_consensus + RANSAC_LIST))
			/ ((float) NB_LINES_SAMPLED);
	//printf("Confidence %f \n", confidence);

	return confidence;
}

#define SCORE_THRESHOLD 200
#define WIDTH_THRESHOLD 100
float detect_line(Mat & img, curve * l, point * pts, int * nb_pts) {
	int i, j;
	(*nb_pts) = 0;
	int * sampled_lines = (int *) malloc(img.cols * sizeof(int));
	srand(time(NULL));
	for (i = 0; i < NB_LINES_SAMPLED; i++) {
		kernel_horiz(img, sampled_lines,
				(SAMPLES_FIRST_LINE + i * (SAMPLES_SPACE)));
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
			pts[(*nb_pts)].y = (SAMPLES_FIRST_LINE + i * (SAMPLES_SPACE));
			(*nb_pts)++;
		}
	}
	free(sampled_lines);
	for (i = 0; i < (*nb_pts); i++) {
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

float steering_from_curve(curve * c, float x_lookahead, float * y_lookahead) {
	(*y_lookahead) = 0;
	int i;
	for (i = 0; i < POLY_LENGTH; i++) {
		(*y_lookahead) += c->p[i] * pow(x_lookahead, i);
	}
	float D_square = pow(x_lookahead, 2) + pow((*y_lookahead), 2);
	float r = D_square / (2.0 * (*y_lookahead));
	float curvature = 1.0 / r;
	return curvature;
}

int detect_line_test(int argc, char ** argv) {
	int i, nb_pts;
	float x, y, u, v;
	curve detected;
	point pts[NB_LINES_SAMPLED];
	if (argc < 2) {
		printf("Requires image path \n");
		exit(-1);
	}
	calc_ct(camera_pose, K, cam_to_bot_in_world, cam_ct); //compute projection matrix from camera coordinates to world coordinates
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
		circle(line_image, Point((int) u, (int) v), 1, Scalar(0, 0, 0, 0), 4, 8,
				0);
	}

	float x_lookahead = 150.0;
	float y_lookahead;
	float steering = steering_from_curve(&detected, x_lookahead, &y_lookahead);
	cout << "steering is " << steering << endl;
	//need to find the correct value
	float angle_from_steering = steering * STEER_P;
	float speed_from_steering = 1.0 - (angle_from_steering*SPEED_DEC)  ;


	/*pixel_to_ground_plane(cam_ct, 320, 479, &x, &y);
	 cout << x << ", " << y << endl;
	 */
	ground_plane_to_pixel(cam_ct, (double) x_lookahead, (double) y_lookahead,
			&u, &v);
	distort_radial(K, u, v, &u, &v, radial_distort, 2);
	//this u,v should be distorded  using the distortion function ...
	circle(line_image, Point((int) u, (int) v), 4, Scalar(255, 0, 0, 0), 4, 8,
			0);
	imshow("orig", line_image);
	imshow("map", map_image);
	waitKey(0);
	return 0;
}