#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "opencv2/highgui/highgui_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/core/types_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "interpolate.h"

char line_detection_kernel[9] = { -1, 0, 1, -1, 0, 1, -1, 0, 1 };

#define IMG_HEIGHT 240
#define SAMPLES_FIRST_LINE 8
#define SAMPLES_LAST_LINE IMG_HEIGHT
#define NB_LINES_SAMPLED 16
#define SAMPLES_SPACE ((SAMPLES_LAST_LINE - SAMPLES_FIRST_LINE)/NB_LINES_SAMPLED)

float rand_a_b(int a, int b) {
	//return ((rand() % (b - a) + a;
	float rand_0_1 = (((float) rand()) / ((float) RAND_MAX));
	return (rand_0_1 * (b - a)) + a;
}

#define POLY_LENGTH 3
typedef struct curve {
	float p[POLY_LENGTH];
	float max_x;
	float min_x;
} curve;

void kernel_line(IplImage * img, char * kernel, int * kernel_response, int v) {
	unsigned int i;
	kernel_response[0] = 0;
	unsigned char * upixel_ptr = (unsigned char *) img->imageData;
	for (i = 1; i < (img->width - 1); i++) {
		int response = 0;
		response += kernel[0] * upixel_ptr[(v - 1) * img->widthStep + (i - 1)];
		response += kernel[1] * upixel_ptr[(v - 1) * img->widthStep + (i)];
		response += kernel[2] * upixel_ptr[(v - 1) * img->widthStep + (i + 1)];

		response += kernel[3] * upixel_ptr[(v) * img->widthStep + (i - 1)];
		response += kernel[4] * upixel_ptr[(v) * img->widthStep + (i)];
		response += kernel[5] * upixel_ptr[(v) * img->widthStep + (i + 1)];

		response += kernel[6] * upixel_ptr[(v + 1) * img->widthStep + (i - 1)];
		response += kernel[7] * upixel_ptr[(v + 1) * img->widthStep + (i)];
		response += kernel[8] * upixel_ptr[(v + 1) * img->widthStep + (i + 1)];

		kernel_response[i] = response;

	}
	kernel_response[i] = 0;
}

void kernel_horiz(IplImage * img, int * kernel_response, int v) {
	unsigned int i;
	kernel_response[0] = 0;
	unsigned char * upixel_ptr = (unsigned char *) img->imageData;
	for (i = 1; i < (img->width - 1); i++) {
		int response = 0;
		response -= upixel_ptr[(v - 1) * img->widthStep + (i - 1)];
		response -= upixel_ptr[(v) * img->widthStep + (i - 1)];
		response -= upixel_ptr[(v + 1) * img->widthStep + (i - 1)];

		response += upixel_ptr[(v - 1) * img->widthStep + (i + 1)];
		response += upixel_ptr[(v) * img->widthStep + (i + 1)];
		response += upixel_ptr[(v + 1) * img->widthStep + (i + 1)];

		kernel_response[i] = response;

	}
	kernel_response[i] = 0;
}

#define RANSAC_LIST (POLY_LENGTH)
#define RANSAC_NB_TEST 8
#define RANSAC_INLIER_LIMIT 4.0
#define NB_RANSAC_TESTS 8
float fit_line(float * pts, unsigned int nb_pts, curve * l) {
	int i, j;
	float * x = malloc(nb_pts * sizeof(float));
	float * y = malloc(nb_pts * sizeof(float));
	char * used = malloc(NB_LINES_SAMPLED * sizeof(char));
	unsigned char * inliers = malloc(NB_LINES_SAMPLED * sizeof(char));
	unsigned char * max_inliers = malloc(NB_LINES_SAMPLED * sizeof(char));
	float fct_params[POLY_LENGTH];
	int max_consensus = 0;
	for (i = 0; i < RANSAC_NB_TEST; i++) {
		int pt_index = 0;
		int nb_consensus = 0;
		int nb_tests = 0;
		int idx;
		float max_x_temp = 0;
		float min_x_temp = 3000.; //arbitrary ...
		memset(used, 0, NB_LINES_SAMPLED * sizeof(char)); //zero used index
		while (pt_index < RANSAC_LIST) {
			//Select set of samples, with distance constraint
			idx = rand_a_b(0, NB_LINES_SAMPLED + 1);
			if (used[idx] != 0 || pts[idx] < 0. || idx >= NB_LINES_SAMPLED)
				continue;
			int closest_neighbour = NB_LINES_SAMPLED;
			for (j = 0; j < NB_LINES_SAMPLED; j++) {
				int idx_to_test = (idx + j) % NB_LINES_SAMPLED;
				if (used[idx_to_test] == 1
						&& abs(idx_to_test - idx) < closest_neighbour) {
					closest_neighbour = abs(idx_to_test - idx);
				}
			}

			if (closest_neighbour > 1) {
				y[pt_index] = (SAMPLES_FIRST_LINE + idx * (SAMPLES_SPACE));
				x[pt_index] = pts[idx];
				if (x[pt_index] > max_x_temp)
					max_x_temp = x[pt_index];
				used[idx] = 1;
				inliers[nb_consensus] = pt_index;
				nb_consensus++;
				pt_index++;
				nb_tests++;
			}
		}
		//From initial set, compute polynom
		compute_interpolation(x, y, fct_params, POLY_LENGTH, pt_index);
		while (1) {
			do {
				idx = rand_a_b(0, NB_LINES_SAMPLED);
			} while (used[idx] == 1 || pts[idx] < 0.);
			used[idx] = 1;

			float idx_y = (SAMPLES_FIRST_LINE + idx * (SAMPLES_SPACE));
			float resp = 0.;
			for (j = 0; j < pt_index; j++) {
				resp += fct_params[j] * pow(pts[idx], j);
			}
			float error = abs(resp - idx_y);
			if (error < RANSAC_INLIER_LIMIT) {
				inliers[nb_consensus] = idx;
				nb_consensus++;
				if (pts[idx] > max_x_temp)
					max_x_temp = pts[idx];
				if (pts[idx] < min_x_temp)
					min_x_temp = pts[idx];
			}
			if (nb_consensus > max_consensus) {
				max_consensus = nb_consensus;
				memcpy(max_inliers, inliers, nb_consensus * sizeof(char));
				l->max_x = max_x_temp;
				l->min_x = min_x_temp;
			}
			nb_tests++;
			if (nb_tests >= nb_pts) {
				break;
			}
		}

	}
	for (i = 0; i < max_consensus; i++) {
		int idx = max_inliers[i];
		y[i] = (SAMPLES_FIRST_LINE + idx * (SAMPLES_SPACE));
		x[i] = pts[idx];
	}
	compute_interpolation(x, y, l->p, POLY_LENGTH, max_consensus);
	//trying to use first last and middle point ...

	/*y[0] = (SAMPLES_FIRST_LINE);
	 x[0] = pts[0];
	 y[1] = (SAMPLES_FIRST_LINE+(NB_LINES_SAMPLED-1)*SAMPLES_SPACE);
	 x[1] = pts[NB_LINES_SAMPLED-1];
	 y[2] = (SAMPLES_FIRST_LINE+(NB_LINES_SAMPLED/2)*SAMPLES_SPACE);
	 x[2] = pts[NB_LINES_SAMPLED/2] ;
	 compute_interpolation(x, y, l->p, POLY_LENGTH, 3);
	 l->min_x = pts[NB_LINES_SAMPLED-1] ;
	 l->max_x = pts[0];*/

	printf("Max consensus %d \n", max_consensus);
	printf("%f + %f*x + %f*x^2 \n", l->p[0], l->p[1], l->p[2]);

	free(x);
	free(y);
	free(used);
	float confidence = ((float) (max_consensus + RANSAC_LIST))
			/ ((float) NB_LINES_SAMPLED);
	printf("Confidence %f \n", confidence);

	return confidence;
}

float detect_line(IplImage * img, curve * l, float * pts) {
	unsigned int i, j;
	int nb_pts = 0;
	int * sampled_lines = malloc(img->width * sizeof(int));
	srand(time(NULL));
	for (i = 0; i < NB_LINES_SAMPLED; i++) {
		/*kernel_line(img, line_detection_kernel, sampled_lines,
		 (SAMPLES_FIRST_LINE + i * (SAMPLES_SPACE)));*/
		kernel_horiz(img, sampled_lines,
				(SAMPLES_FIRST_LINE + i * (SAMPLES_SPACE)));
		int max = 0, min = 0, max_index, min_index, sig = 0;
		for (j = 1; j < (img->width - 1); j++) {
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
		if (sig == 1) {
			pts[i] = ((float) (max_index + min_index)) / 2.;
			nb_pts++;
		} else {
			pts[i] = -1; //not found ...
		}
	}
	free(sampled_lines);
	return fit_line(pts, nb_pts, l);
	//TODO : for each detected point, compute its projection in the world frame instead of plain image coordinates
	//

}

float steering_from_curve(curve * c, float * s) {
	float x_lookahead, y_lookahead;

	float D_square = pow(x_lookahead, 2) + pow(y_lookahead, 2);
	float r = D_square / (2.0 * x_lookahead);
	float curvature = 1.0 / r;
	return curvature;
}

//Compute the line equation in the ground plane (need a calibrated camera system ...)
//Compute an interception curve in the ground plane
//Compute steering from interception curve and apply to vehicle (compute speed based on steering to avoid drift)

int main(void) {
	int i;
	curve detected;
	float pts[NB_LINES_SAMPLED];
	IplImage * line_image =
			cvLoadImage(
					"/home/jpiat/development/SOFTWARE/pi_line_follower/img_test/curve_view.png",
					CV_LOAD_IMAGE_GRAYSCALE);
	init_profile(0);
	start_profile(0);
	detect_line(line_image, &detected, pts);
	end_profile(0);
	print_profile_time("Took :", 0);
	for (i = 0; i < NB_LINES_SAMPLED; i++) {
		cvLine(line_image,
				cvPoint(0, (SAMPLES_FIRST_LINE + i * (SAMPLES_SPACE))),
				cvPoint(line_image->width - 1,
						(SAMPLES_FIRST_LINE + i * (SAMPLES_SPACE))),
				cvScalar(255, 255, 255, 0), 1, 8, 0);
		if (pts[i] > 0) {
			cvCircle(line_image,
					cvPoint((int) (pts[i]),
							(SAMPLES_FIRST_LINE + i * (SAMPLES_SPACE))), 4,
					cvScalar(0, 0, 0, 0), 4, 8, 0);
		}
	}

	for (i = 0; i <= detected.max_x; i += 10) {
		float resp1 = 0., resp2 = 0.;
		int j;
		for (j = 0; j < POLY_LENGTH; j++) {
			resp1 += detected.p[j] * pow(i, j);
			resp2 += detected.p[j] * pow((i + 10), j);
		}
		cvLine(line_image, cvPoint(i, resp1), cvPoint(i + 10, resp2),
				cvScalar(255, 255, 255, 0), 1, 8, 0);
	}

	cvShowImage("orig", line_image);
	cvWaitKey(0);
	return 0;
}