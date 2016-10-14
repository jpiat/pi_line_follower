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

#define NB_LINES_HORIZ_SAMPLING 8
#define NB_LINES_SAMPLED 28
#define SAMPLE_SPACING_MM 30.0

float posx_samples_world[NB_LINES_SAMPLED];
unsigned int posv_samples_cam[NB_LINES_SAMPLED];

float * x;
float * y;
char * used;
unsigned char * inliers;
unsigned char * max_inliers;

float rand_a_b(int a, int b) {
	//return ((rand() % (b - a) + a;
	float rand_0_1 = (((float) rand()) / ((float) RAND_MAX));
	return (rand_0_1 * (b - a)) + a;
}

float poly_at(curve * c, float x) {
	int i;
	float resp = 0.;
	for (i = 0; i < POLY_LENGTH; i++) {
		resp += c->p[i] * pow(x, i);
	}
	return resp;
}

float poly_deriv_at(curve * c, float derivx, float x) {
	int i;
	float resp = 0.;
	for (i = 1; i < POLY_LENGTH; i++) {
		resp += c->p[i] * pow(derivx, (i - 1));
	}
	resp = (resp * (x - derivx)) + poly_at(c, derivx);
	return resp;
}

void kernel_horiz(Mat & img, int * kernel_response, unsigned int v,
		unsigned int u_start, unsigned int u_end) {
	int i;
	memset(kernel_response, 0, sizeof(int) * img.cols);
	unsigned char * upixel_ptr = (unsigned char *) img.data;
	for (i = u_start; i < u_end && i < img.cols; i++) {
		int response_x = 0;
		response_x -= upixel_ptr[(v - 1) * img.step + (i - 1)];
		response_x -= 2 * upixel_ptr[(v) * img.step + (i - 1)];
		response_x -= upixel_ptr[(v + 1) * img.step + (i - 1)];

		response_x += upixel_ptr[(v - 1) * img.step + (i + 1)];
		response_x += 2 * upixel_ptr[(v) * img.step + (i + 1)];
		response_x += upixel_ptr[(v + 1) * img.step + (i + 1)];

		kernel_response[i] = response_x;

	}
}

//This is the most time consuming function for now
#define NB_LOOP_DISTANCE 4
inline float distance_to_curve(curve * l, float x, float y) {
	float resp_1 = 0., resp_2 = 0.;
	float current_x = x, current_x_1, current_x_2;
	int i;
	float error_1 = 0, error_2 = 0, last_error = 320000.;
	float increment = 1.;
	for (i = 0; i < NB_LOOP_DISTANCE; i++) {
		float current_x_1 = current_x - increment, current_x_2 = current_x
				+ increment;
		resp_1 = poly_at(l, current_x_1);
		resp_2 = poly_at(l, current_x_2);
		error_1 = sqrt(
				((current_x_1 - x) * (current_x_1 - x))
						+ ((resp_1 - y) * (resp_1 - y)));
		error_2 = sqrt(
				((current_x_2 - x) * (current_x_2 - x))
						+ ((resp_2 - y) * (resp_2 - y)));
		if (error_1 < error_2 && error_1 < last_error) {
			current_x = current_x_1;
			last_error = error_1;
		} else if (error_2 <= error_1 && error_2 < last_error) {
			current_x = current_x_2;
			last_error = error_2;
		} else {
			increment = increment / 2.;
		}
	}
	return last_error;
}

#define RANSAC_LIST (POLY_LENGTH)
#define RANSAC_NB_LOOPS NB_LINES_SAMPLED
#define RANSAC_INLIER_LIMIT 5.0
float fit_line(point * pts, unsigned int nb_pts, curve * l) {
	int i;
//Should move dynamic memory allocation to static
	int max_consensus = 0;
	for (i = 0; i < RANSAC_NB_LOOPS; i++) {
		int pt_index = 0;
		int nb_consensus = 0;
		unsigned int idx = 0;
		float max_x_temp = 0;
		float min_x_temp = 300000.; //arbitrary ...
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
		if (idx < 0 || idx > NB_LINES_SAMPLED) {
			cout << "idx error" << endl;
		} else {
			y[i] = pts[idx].y;
			x[i] = pts[idx].x;
		}
	}
//Should recompute the curve interpolation with inliers
	compute_interpolation(x, y, l->p, POLY_LENGTH, max_consensus);

//printf("Max consensus %d \n", max_consensus);
	/*
	 printf("%f + %f*x + %f*x^2 \n", l->p[0], l->p[1], l->p[2]);
	 */
	float confidence = ((float) (max_consensus)) / ((float) nb_pts);
//printf("Confidence %f \n", confidence);

	return confidence;
}

#define SCORE_THRESHOLD 100
#define WIDTH_THRESHOLD 100
void extract_line_pos(int * horizontal_gradient, unsigned int line_start,
		unsigned int line_stop, float * line, int * nb_lines) {
	int j;
	int max = 0, min = 0, max_index = 0, min_index = 0;
	int scores[8]; //would need to be dynamically allocated
	memset(scores, 0, 8 * sizeof(int));
	int max_nb_lines = (*nb_lines);
	int new_detected = 0;
	(*nb_lines) = 0;
	for (j = line_start; j < line_stop; j++) {
		//Track is white on black, so we expect maximum gradient then minimum
		if (horizontal_gradient[j] < min) {
			if (max > 0) { //we have a signature ...
				min = horizontal_gradient[j];
				min_index = j;
				scores[(*nb_lines)] = abs(min) + abs(max);
				if (scores[(*nb_lines)] > SCORE_THRESHOLD) {
					(*line) = ((float) (max_index + min_index)) / 2.;
					if (new_detected == 0)
						(*nb_lines)++; //first time the line is discovered
				}
				new_detected = 1;
			}
		}
		if (horizontal_gradient[j] > max) {
			if (min < 0) {
				min = 0;
				max = horizontal_gradient[j];
				max_index = j;
				new_detected = 0;
				//start of a new line ...
			} else {
				max = horizontal_gradient[j];
				max_index = j;
			}
		}
	}

}

float detect_line(Mat & img, curve * l, point * pts, int * nb_pts, int track) {
	int i;
	(*nb_pts) = 0;
	int * sampled_lines = (int *) malloc(img.cols * sizeof(int));
	srand(time(NULL));
	unsigned int initial_search_start_u = 0;
	unsigned int initial_search_stop_u = img.cols;
	if (track == 1) {
		//TODO: take curve equation
		double x = (SAMPLE_SPACING_MM);
		double y = poly_at(l, x);
		float u, v;
		double y_1 = y - 150., y_2 = y + 150.;
		ground_plane_to_pixel(cam_ct, x, y_1, &u, &v);
		distort_radial(K, u, v, &u, &v, radial_distort, POLY_DISTORT_SIZE);
		initial_search_start_u = (u >= 0) ? round(u) : 0;
		ground_plane_to_pixel(cam_ct, x, y_2, &u, &v);
		distort_radial(K, u, v, &u, &v, radial_distort, POLY_DISTORT_SIZE);
		initial_search_stop_u = (u >= 0) ? round(u) : 0;

		if (initial_search_stop_u < initial_search_start_u) {
			unsigned int temp = initial_search_stop_u;
			initial_search_stop_u = initial_search_start_u;
			initial_search_start_u = temp;
		}
		if (initial_search_stop_u >= img.cols)
			initial_search_stop_u = img.cols;
		//compute curve position at first sample
		//limit search space {initial_search_start_u, initial_search_stop_u}
	}
	for (i = 0; i < NB_LINES_HORIZ_SAMPLING; i++) {
		kernel_horiz(img, sampled_lines, posv_samples_cam[i],
				initial_search_start_u, initial_search_stop_u);
		float line_pos;
		int nb_lines = 1;
		extract_line_pos(sampled_lines, initial_search_start_u,
				initial_search_stop_u, &line_pos, &nb_lines);
		if (nb_lines > 0) {
			pts[(*nb_pts)].x = line_pos;
			pts[(*nb_pts)].y = posv_samples_cam[i];
			(*nb_pts)++;
		}
	}

	for (i = 0; i < (*nb_pts); i++) {
		undistort_radial(K, pts[i].x, pts[i].y, &(pts[i].x), &(pts[i].y),
				radial_undistort, POLY_UNDISTORT_SIZE);
		pixel_to_ground_plane(cam_ct, pts[i].x, pts[i].y, &(pts[i].x),
				&(pts[i].y));
	}

	if ((*nb_pts) > ((POLY_LENGTH * 2.0) - 1)) {
		float confidence = fit_line(pts, (*nb_pts), l);
		for (i = NB_LINES_HORIZ_SAMPLING; i < NB_LINES_SAMPLED; i++) {
			float u, v;
			float old_l_max_x = l->max_x;
			float y = poly_deriv_at(l, (l->max_x - SAMPLE_SPACING_MM),
					(i * SAMPLE_SPACING_MM));
			ground_plane_to_pixel(cam_ct, (i * SAMPLE_SPACING_MM), y, &u, &v);
			if (u < 0 || u >= img.cols || v < 0 || v >= img.rows)
				break;
			kernel_horiz(img, sampled_lines, v, u - 50, u + 50);
			float line_pos;
			int nb_lines = 1;
			extract_line_pos(sampled_lines, u - 50, u + 50, &line_pos,
					&nb_lines);
			if (nb_lines > 0) {
				undistort_radial(K, line_pos, v, &(pts[(*nb_pts)].x),
						&(pts[(*nb_pts)].y), radial_undistort,
						POLY_UNDISTORT_SIZE);
				pixel_to_ground_plane(cam_ct, pts[(*nb_pts)].x,
						pts[(*nb_pts)].y, &(pts[(*nb_pts)].x),
						&(pts[(*nb_pts)].y));
				(*nb_pts)++;
				confidence = fit_line(pts, (*nb_pts), l);
			} else {
				break;
			}
		}
		free(sampled_lines);
		return confidence;
	} else {
		free(sampled_lines);
		return 0.;
	}
	free(sampled_lines);
	return 0.;
//return 0.;
//TODO : for each detected point, compute its projection in the world frame instead of plain image coordinates
//

}

void init_line_detector() {
	int i;
	float u, v;
	calc_ct(camera_pose, K, cam_to_bot_in_world, cam_ct); //compute projection matrix from camera coordinates to world coordinates
//Sampling world frame and projecting into camera frame
	for (i = 0; i < NB_LINES_SAMPLED; i++) {
		posx_samples_world[i] = (i + 1) * SAMPLE_SPACING_MM;
		ground_plane_to_pixel(cam_ct, posx_samples_world[i], 0, &u, &v);
		distort_radial(K, u, v, &u, &v, radial_distort, POLY_DISTORT_SIZE);
		posv_samples_cam[i] = v;
	}
	x = (float *) malloc(NB_LINES_SAMPLED * sizeof(float));
	y = (float *) malloc(NB_LINES_SAMPLED * sizeof(float));
	used = (char *) malloc(NB_LINES_SAMPLED * sizeof(char));
	inliers = (unsigned char *) malloc(
	NB_LINES_SAMPLED * sizeof(char));
	max_inliers = (unsigned char *) malloc(
	NB_LINES_SAMPLED * sizeof(char));
}

void close_line_detector() {
	free(x);
	free(y);
	free(used);
	free(inliers);
	free(max_inliers);
}

int detect_line_test(int argc, char ** argv) {
	int i, nb_pts;
	int image_index = 0 ;
	float x, y, u, v, y_lookahead, speed_factor, curvature;
	curve detected;
	point pts[NB_LINES_SAMPLED];
	if (argc < 2) {
		printf("Requires image path \n");
		exit(-1);
	}
	char * image_path = (char *) malloc(256); //
	init_line_detector();
	while (1) {
		Mat line_image;
		sprintf(image_path, argv[1], image_index);
		line_image = imread(image_path, IMREAD_GRAYSCALE);
		if(line_image.data == NULL) break ;
		Mat map_image(480, 480,
		CV_8UC1, Scalar(255));
		Mat view_img(line_image.cols, line_image.rows, CV_8UC3);
		cvtColor(line_image, view_img, CV_GRAY2RGB);
		tic
		;
		//for (i = 0; i < 1000; i++) {
		detect_line(line_image, &detected, pts, &nb_pts, 0);
//		detect_line(line_image, &detected, pts, &nb_pts, 1); //now track
		//}
		toc
		;
		for (i = 0; i < nb_pts; i++) {
			ground_plane_to_pixel(cam_ct, (double) pts[i].x, (double) pts[i].y,
					&u, &v);
			distort_radial(K, u, v, &u, &v, radial_distort, POLY_DISTORT_SIZE);
			circle(view_img, Point((int) u, (int) v), 1, Scalar(0, 0, 255, 0),
					4, 8, 0);

			circle(map_image, Point(pts[i].x, (pts[i].y + map_image.cols / 2)),
					1, Scalar(0, 0, 0, 0), 4, 8, 0);
		}

		for (x = detected.min_x; x <= detected.max_x ; x += 0.1) {
			float resp = 0.;
			for (i = 0; i < POLY_LENGTH; i++) {
				resp += detected.p[i] * pow(x, i);
			}
			ground_plane_to_pixel(cam_ct, (double) x, (double) resp, &u, &v);
			distort_radial(K, u, v, &u, &v, radial_distort, POLY_DISTORT_SIZE);
			if (u > line_image.cols || u < 0 || v > line_image.rows || v < 0)
				break;
			circle(view_img, Point((int) u, (int) v), 1, Scalar(0, 255, 0, 0),
					1, 8, 0);
		}

		curvature = steering_speed_from_curve(&detected, 300.0, &y_lookahead,
				&speed_factor);
		pixel_to_ground_plane(cam_ct, IMAGE_WIDTH / 2, IMAGE_HEIGHT - 1, &x,
				&y);
		cout << x << ", " << y << endl;

		ground_plane_to_pixel(cam_ct, (double) 300, (double) y_lookahead, &u,
				&v);
		circle(line_image, Point((int) u, (int) v), 1, Scalar(255, 0, 0, 0), 2,
				8, 0);
		cout << "Poly : y" << detected.p[0] << ", " << detected.p[1] << ", "
				<< detected.p[2] << endl;
		cout << "Curvature " << curvature << endl;
		cout << "Speed factor " << speed_factor << endl;
		cout << "Y lookahead " << y_lookahead << endl;
		imshow("orig", view_img);
		imshow("map", map_image);
		waitKey(0);
		image_index ++ ;
	}
}
