#include <visual_odometry.hpp>

#define tic      double tic_t = clock();
#define toc      std::cout << (clock() - tic_t)/CLOCKS_PER_SEC \
                           << " seconds" << std::endl;

typedef char binary_descriptor[DESCRIPTOR_LENGTH / 8];

typedef unsigned char comp_vect[4];

typedef struct feature {
	xy pos;
	binary_descriptor * desc;
} feature;

typedef struct descriptor_stack {
	unsigned int nb;
	unsigned int stack_size;
	feature ** stack;
} descriptor_stack;

descriptor_stack * current_stack = NULL;
descriptor_stack * last_stack = NULL;

unsigned int nb_descriptors_in_stack = 0;
comp_vect * briefPattern;

unsigned int first_line_to_sample, last_line_to_sample;

void init_stack(descriptor_stack * stack, unsigned int stack_size) {
	stack->stack = (feature **) malloc(stack_size * sizeof(feature*));
	stack->nb = 0;
	stack->stack_size = stack_size;
}

int push_stack(descriptor_stack * stack, feature * desc) {
	if (stack->nb >= stack->stack_size)
		return 0;
	stack->stack[stack->nb] = desc;
	stack->nb++;
	return 1;
}

int pop_stack(descriptor_stack * stack, feature ** desc) {
	if (stack->nb == 0)
		return 0;
	stack->nb--;
	(*desc) = stack->stack[stack->nb];
	return 1;
}

int get_stack_at(descriptor_stack * stack, unsigned int index,
		feature ** desc) {
	if (index >= stack->nb)
		return 0;
	(*desc) = stack->stack[index];
	return 1;
}

int set_stack_at(descriptor_stack * stack, unsigned int index, feature * desc) {
	if (index >= stack->nb)
		return 0;
	stack->stack[index] = desc;
	return 1;
}

int rand_a_b_brief(int a, int b) {
	//return ((rand() % (b - a) + a;
	float rand_0_1 = (((float) rand()) / ((float) RAND_MAX));
	return (rand_0_1 * (b - a)) + a;
}

comp_vect * initBriefPattern(comp_vect * pattern, int size) {
	int i;
	if (!pattern) {
		pattern = (comp_vect *) malloc(size * sizeof(comp_vect));
	}
	srand(time(NULL));
	int dist_max = sqrt(pow(DESCRIPTOR_WINDOW, 2) + pow(DESCRIPTOR_WINDOW, 2));
	int dist_min = dist_max / 2; // could be altered to have increasing resolution in the descriptor
	for (i = 0; i < size; i++) {
		pattern[i][0] = rand_a_b_brief(0, DESCRIPTOR_WINDOW);
		pattern[i][1] = rand_a_b_brief(0, DESCRIPTOR_WINDOW);
		if (i % 32 == 0 && dist_min > 4) {
			dist_min = dist_min / 2;
			if (dist_min < 4)
				dist_min = 4;
			dist_max = dist_max * 0.75;
		}
		float dec_c, dec_l;
		int pos_l, pos_c;
		do {
			double dist = (((double) rand()) / ((double) RAND_MAX))
					* (dist_max - dist_min) + dist_min;
			double angle = (((double) rand()) / ((double) RAND_MAX)) * 2.0
					* M_PI;

			dec_c = (int) (cos(angle) * (double) dist);
			dec_l = (int) (sin(angle) * (double) dist);

			pos_l = ((int) pattern[i][0]) + dec_l;
			pos_c = ((int) pattern[i][1]) + dec_c;

		} while (pos_l < 0 || pos_c < 0 || pos_l >= DESCRIPTOR_WINDOW
				|| pos_c >= DESCRIPTOR_WINDOW);

		pattern[i][2] = pos_l;
		pattern[i][3] = pos_c;
	}
	return pattern;
}

binary_descriptor * compute_descriptor(Mat &img, xy pos) {
	unsigned int i, byte_index = 0, bit_count = 0;
	int x_top_left = pos.x - (DESCRIPTOR_WINDOW / 2);
	int y_top_left = pos.y - (DESCRIPTOR_WINDOW / 2);
	binary_descriptor * desc;
	desc = (binary_descriptor *) malloc(sizeof(binary_descriptor));
	for (i = 0; i < DESCRIPTOR_LENGTH; i++) {
		int lx = briefPattern[i][0];
		int px = briefPattern[i][1];
		int ly = briefPattern[i][2];
		int py = briefPattern[i][3];
		(*desc)[byte_index] = (*desc)[byte_index] << 1;
		unsigned int gauss_value_p = img.at<unsigned char>(lx + y_top_left,
				px + x_top_left);
		unsigned int gauss_value_n = img.at<unsigned char>(ly + y_top_left,
				py + x_top_left);
		if (gauss_value_n > gauss_value_p) {
			(*desc)[byte_index] |= 0x1;
		} else {
			(*desc)[byte_index] &= ~0x01;
		}
		bit_count++;
		if (bit_count >= 8) {
			bit_count = 0;
			byte_index++;
		}
	}
	return desc;
}

unsigned int get_match_score(binary_descriptor * d0, binary_descriptor * d1) {
	unsigned long int i, dist = 0; // j,
	unsigned long int * bits1_32 = (unsigned long int *) (*d0);
	unsigned long int * bits2_32 = (unsigned long int *) (*d1);

	for (i = 0; i < (DESCRIPTOR_LENGTH / 32); i++) {
		unsigned long int xored = bits1_32[i] ^ bits2_32[i];
		dist += __builtin_popcount(xored);
		if (dist > DESCRIPTOR_MATCH_THRESHOLD)
			return dist; //don't try harder if this not a match
	}
	return dist;
}

void showPatch(unsigned char * img, char * title, unsigned int w,
		unsigned int h, unsigned int offset_x, unsigned int offset_y) {
	Mat image(Size(w, h), CV_8UC1);
	image.data = (unsigned char *) img;
	image.step = w;
	Mat patchImageUp(Size(20 * DESCRIPTOR_WINDOW, 20 * DESCRIPTOR_WINDOW),
	CV_8UC1);

	Mat patchImage(image,
			Rect(offset_x - (DESCRIPTOR_WINDOW / 2),
					offset_y - (DESCRIPTOR_WINDOW / 2), DESCRIPTOR_WINDOW,
					DESCRIPTOR_WINDOW));
	resize(patchImage, patchImageUp, patchImageUp.size(), 0, 0, INTER_CUBIC);
	imshow(title, patchImageUp);
	waitKey(0);
	patchImageUp.release();
}

#define HOUGH_X 30
#define HOUGH_Y 30
#define SPEED_X_MAX 5000.0
#define SPEED_X_MIN 0.0
#define SPEED_Y_MAX 500.0
#define SPEED_Y_MIN -500.0
#define SPEED_X_STEP ((SPEED_X_MAX - SPEED_X_MIN)/((float) HOUGH_X))
#define SPEED_Y_STEP ((SPEED_Y_MAX - SPEED_Y_MIN)/((float) HOUGH_Y))

//hough vote to get average speed for x and y
int hough_votes(fxy * flow, unsigned int nb_flows, float * speed_x,
		float * speed_y) {
	int i, j;
	int max = 0, max_index = -1;
	float * vote_space = (float *) malloc(HOUGH_X * HOUGH_Y * sizeof(int));
	float * vote_space_pop = (float *) malloc(nb_flows * sizeof(unsigned int));
	memset(vote_space, 0, sizeof(int) * HOUGH_X * HOUGH_Y);
	(*speed_x) = 0.0;
	(*speed_y) = 0.0;
	int nb_pop_max = 0;
	for (i = 0; i < nb_flows; i++) {
//		cout << "speed x is " << flow[i].x << endl;
//		 cout << "speed y is " << flow[i].y << endl;
		int indx = (flow[i].x - SPEED_X_MIN) / SPEED_X_STEP;
		int indy = (flow[i].y - SPEED_Y_MIN) / SPEED_Y_STEP;
//		cout << "vote index : "<<indx << ", " << indy << endl ;
		if (indx > HOUGH_X || indy > HOUGH_Y || indx < 0 || indy < 0)
			continue; //does not fit the model
		vote_space[(indy * HOUGH_X) + indx]++;
		vote_space_pop[i] = (indy * HOUGH_X) + indx;
		if (vote_space[(indy * HOUGH_X) + indx] > max) {
			max = vote_space[(indy * HOUGH_X) + indx];
			max_index = (indy * HOUGH_X) + indx;
		}
	}
	if (max_index >= 0 && max > 4) {
		for (i = 0; i < nb_flows; i++) {
			if (vote_space_pop[i] == max_index) {
				nb_pop_max++;
				(*speed_x) += flow[i].x;
				(*speed_y) += flow[i].y;
			}
		}
//	cout << " max pop is " << nb_pop_max << endl ;
		if (nb_pop_max > 4) {
			(*speed_x) /= nb_pop_max;
			(*speed_y) /= nb_pop_max;
		}
	}
	free(vote_space);
	free(vote_space_pop);
	return nb_pop_max;
}

#define FAST_THRESHOLD 90
int estimate_ground_speeds(Mat & img, fxy * speed) {
	unsigned int i, j;
	int nb_corners;
	xy* corners;
	fxy * flow_vectors = (fxy*) malloc(STACK_SIZE * sizeof(fxy));
	int flow_vector_size = 0;

	current_stack = (descriptor_stack *) malloc(sizeof(descriptor_stack));
	corners = fast9_detect_nonmax(
			(img.data + (first_line_to_sample * img.step)), img.cols,
			(last_line_to_sample - first_line_to_sample), img.step,
			FAST_THRESHOLD, &nb_corners);
	init_stack(current_stack, STACK_SIZE);
#ifdef DEBUG
	cout << "found " << nb_corners << " corners" << endl;
#endif
	for (i = 0; i < nb_corners; i++) {
		corners[i].y += first_line_to_sample;
		feature * current = (feature *) malloc(sizeof(feature));
		current->pos.x = corners[i].x;
		current->pos.y = corners[i].y;
		/*	showPatch(img.data, "patch", img.cols, img.rows,
		 corners[i].x, corners[i].y);*/
		current->desc = compute_descriptor(img, corners[i]);
#ifdef DEBUG
		circle(img, Point(current->pos.x, current->pos.y), 2,
				Scalar(0, 0, 0, 0), 2, 8, 0);
#endif
		if (push_stack(current_stack, current) == 0)
			break;
	}
	free(corners); //corners where copied in feature, it can be freed
	if (last_stack != NULL) {
		for (i = 0; i < current_stack->nb; i++) {
			feature * f0 = NULL;
			unsigned int best_score = DESCRIPTOR_MATCH_THRESHOLD;
			unsigned int best_score_index = 0;
			get_stack_at(current_stack, i, &f0);
			for (j = 0; j < last_stack->nb; j++) {
				feature * f1;
				get_stack_at(last_stack, j, &f1);
				if (f1 == NULL)
					continue;
				unsigned int score = get_match_score(f1->desc, f0->desc);
				if (score < best_score) {
					best_score = score;
					best_score_index = j;
				}
			}
			if (best_score < DESCRIPTOR_MATCH_THRESHOLD) {
				feature * f1;
				get_stack_at(last_stack, best_score_index, &f1);
				//project in robot frame
				float gp0x, gp0y, gp1x, gp1y;
				float ip0x, ip0y, ip1x, ip1y;
#ifdef DEBUG
				line(img, Point(f0->pos.x, f0->pos.y),
						Point(f1->pos.x, f1->pos.y), Scalar(255, 0, 0, 0), 2, 8,
						0);
#endif
				//should distort point before projection
				undistort_radial(K, (float) f0->pos.x, (float) f0->pos.y,
						&(ip0x), &(ip0y), radial_undistort,
						POLY_UNDISTORT_SIZE);
				undistort_radial(K, (float) f1->pos.x, (float) f1->pos.y,
						&(ip1x), &(ip1y), radial_undistort,
						POLY_UNDISTORT_SIZE);
				pixel_to_ground_plane(cam_ct, ip0x, ip0y, &gp0x, &gp0y);
				pixel_to_ground_plane(cam_ct, ip1x, ip1y, &gp1x, &gp1y);
				flow_vectors[flow_vector_size].x = gp1x - gp0x;
				flow_vectors[flow_vector_size].y = gp1y - gp0y;
				flow_vector_size++;
				free(f1->desc);
				free(f1);
				set_stack_at(last_stack, best_score_index, NULL);
			}
		}
		while (last_stack->nb > 0) {
			feature * f1;
			pop_stack(last_stack, &f1);
			if (f1 != NULL) {
				free(f1->desc);
				free(f1);
			}
		}
		free(last_stack);
		last_stack = current_stack;
		if (flow_vector_size > 4) {
			int nb_pop = hough_votes(flow_vectors, flow_vector_size,
					&(speed->x), &(speed->y));
			free(flow_vectors);
			return nb_pop;
		} else {
			free(flow_vectors);
			return 0;
		}
	} else {
		free(flow_vectors);
		last_stack = current_stack;
		return 0; //exchange for the next frame
	}
}

void init_visual_odometry() {
	float u, v;
	briefPattern = initBriefPattern(briefPattern, DESCRIPTOR_LENGTH);
	calc_ct(camera_pose, K, cam_to_bot_in_world, cam_ct); //compute projection matrix from camera coordinates to world coordinates
	ground_plane_to_pixel(cam_ct, 500., 0., &u, &v);
	first_line_to_sample = (unsigned int) v;
	ground_plane_to_pixel(cam_ct, 20., 0., &u, &v);
	last_line_to_sample = (unsigned int) v;
}

int test_estimate_ground_speeds(int argc, char ** argv) {
	if (argc < 3) {
		printf("Requires image path \n");
		exit(-1);
	}
	int success;
	fxy speed;
	init_visual_odometry();
	Mat first_image, second_image;
	first_image = imread(argv[1], IMREAD_GRAYSCALE);
	second_image = imread(argv[2], IMREAD_GRAYSCALE);
	estimate_ground_speeds(first_image, &speed);
	tic
	success = estimate_ground_speeds(second_image, &speed);
	toc
	if (success > 0) {
		cout << success << ", " << speed.x << ", " << speed.y << endl;
	} else {
		cout << "Cannot estimate speed" << endl;
	}

	imshow("first", first_image);
	imshow("second", second_image);
	waitKey(0);
	return 0;
}
