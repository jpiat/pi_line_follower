#include "resampling.h"

double distort_plumb_bob(double xn, double yn, double * xd, double * yd,
		double * poly) {
	double r_square = xn * xn + yn * yn;
	double k = 1.0 + r_square * poly[0] + (r_square * r_square) * poly[1]
			+ (r_square * r_square) * r_square * poly[4];
	double tx = 2 * poly[2] * (xn * yn)
			+ poly[3] * (r_square + (2 * (xn * xn)));
	double ty = 2 * poly[3] * (xn * yn)
			+ poly[2] * (r_square + (2 * (yn * yn)));
	(*xd) = k * xn + tx;
	(*yd) = k * yn + ty;
	return sqrt(pow((*xd) - xn, 2) + pow((*yd) - yn, 2));
}

void homography(char * image_data, int w, int h, char * img_h, double * H,
		double * K1, double * K2) {
	int u, v;
	memset(img_h, 0, w * h);
	//p_a = Hba*p_b -> dans le plan image normalisé
	for (u = 0; u < w; u++) {
		for (v = 0; v < h; v++) {
			double xn = (u - K1[2]) / K1[0]; //vers plan image normalisé avec origine en axe principale et unité métrique
			double yn = (v - K1[5]) / K1[4];

			double xnp = xn * H[0] + yn * H[1] + H[2];
			double ynp = xn * H[3] + yn * H[4] + H[5];
			double wnp = xn * H[6] + yn * H[7] + H[8];

			double xnp_norm = xnp / wnp;
			double ynp_norm = ynp / wnp;

			double xnp_norm_img = xnp_norm * K2[0] + K2[2]; //vers plan image normalisé avec origine en axe principale et unité métrique
			double ynp_norm_img = ynp_norm * K2[4] + K2[5];
			int hu = floor(xnp_norm_img);
			int hv = floor(ynp_norm_img);

			if (hu >= 0 && hv >= 0 && hu < w && hv < h) {

				float pixel_val1, pixel_val2, pixel_val;
				float pixa, pixb, pixc, pixd;

				/*if (xnp_norm_img < hu)
				 cout << "problem" << endl;
				 if (ynp_norm_img < hv)
				 cout << "problem" << endl;*/
				pixa = image_data[(hu) + (hv * w)];
				pixb = image_data[(hu + 1) + (hv * w)];

				pixc = image_data[(hu) + ((hv + 1) * w)];
				pixd = image_data[(hu + 1) + ((hv + 1) * w)];

				pixel_val1 = pixa * (1 - (xnp_norm_img - hu));
				pixel_val1 += pixb * ((xnp_norm_img - hu));

				pixel_val2 = pixc * (1 - (xnp_norm_img - hu));
				pixel_val2 += pixd * ((xnp_norm_img - hu));

				pixel_val = pixel_val1 * (1 - (ynp_norm_img - hv));
				pixel_val += pixel_val2 * (ynp_norm_img - hv);

				img_h[u + (v * w)] = round(pixel_val);
			}
		}
	}
}

void calc_ct_and_H(double * world_to_cam, double * K, double *Ct, double * H) {
	int i;
	struct double_matrix world_to_cam_struct, K_struct, Ct_struct;
	alloc_double_matrix(&world_to_cam_struct, 4, 3, world_to_cam);
	alloc_double_matrix(&K_struct, 3, 3, K);
	alloc_double_matrix(&Ct_struct, 4, 3, Ct);

	double_mat_product(&K_struct, &world_to_cam_struct, &Ct_struct, 0, 0);

	print_double_matrix(Ct_struct, "Ct");
	H[0] = Ct[0];
	H[1] = Ct[1];
	H[2] = Ct[3];
	H[3] = Ct[4];
	H[4] = Ct[5];
	H[5] = Ct[7];
	H[6] = Ct[8];
	H[7] = Ct[9];
	H[8] = Ct[11];
	printf("H =");
	for (i = 0; i < 9; i++)
		printf("%lf, ", H[i]);
}
void pixel_to_ground_plane(double * Ct, double u, double v, float * x,
		float * y) {
	//TODO: replace following with good transformation
	float t = (*x);
	(*x) = (*y);
	(*y) = t;
	return;
	//

	struct double_matrix Ct_struct;
	alloc_double_matrix(&Ct_struct, 4, 3, Ct);

	double a1 = MAT_ELT_AT(Ct_struct, 0, 0) - u * MAT_ELT_AT(Ct_struct, 2, 0);
	double b1 = MAT_ELT_AT(Ct_struct, 0, 1) - u * MAT_ELT_AT(Ct_struct, 2, 1);
	double c1 = MAT_ELT_AT(Ct_struct, 0, 2) - u * MAT_ELT_AT(Ct_struct, 2, 2);

	double a2 = MAT_ELT_AT(Ct_struct, 1, 0) - v * MAT_ELT_AT(Ct_struct, 2, 0);
	double b2 = MAT_ELT_AT(Ct_struct, 1, 1) - v * MAT_ELT_AT(Ct_struct, 2, 1);
	double c2 = MAT_ELT_AT(Ct_struct, 1, 2) - v * MAT_ELT_AT(Ct_struct, 2, 2);

	double d1 = -(MAT_ELT_AT(Ct_struct, 0, 3) - u * MAT_ELT_AT(Ct_struct, 2, 3));
	double d2 = -(MAT_ELT_AT(Ct_struct, 1, 3) - v * MAT_ELT_AT(Ct_struct, 2, 3));

	double b3 = a2 * b1 - a1 * b2;
	double c3 = a2 * c1 - a1 * c2;
	double d3 = a2 * d1 - a1 * d2;

	(*y) = d3 / b3;
	(*x) = (d1 - ((*y) * b1)) / a1;

}

/**
 * Poses are expressed in world frame whose origin is center of the scene on the ground plane
 * n is plane normal in world frame, d is plane distance to origin in world frame
 */
void compute_homography_from_cam_cam_pos(double * C1_pose, double * C2_pose,
		double * n, double d, double * K1, double * K2, double *H) {
	// poses are express as a camera pose |R t| a 4x4 matrix
	//                                    |0 1|

	int i, j;
	struct double_matrix R_C1, R_C2, tC1, tC2, tC2_C2C1, R_C2C1, t_C2_C2C1_nC1t;
	struct double_matrix C1_pose_struct, C2_pose_struct, C1_pose_struct_inv,
			C2C1_pose;
	struct double_matrix nC1, n_struct, dC1_mat, H_struct;
	struct double_matrix K1_struct, K2_struct, K1H, K2_inv;
	double dC1;

	if (alloc_double_matrix(&R_C1, 3, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&R_C2, 3, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&R_C2C1, 3, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&tC2_C2C1, 1, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&nC1, 1, 4, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&tC1, 1, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&tC2, 1, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&dC1_mat, 1, 1, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&t_C2_C2C1_nC1t, 3, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&C1_pose_struct_inv, 4, 4, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&C2C1_pose, 4, 4, NULL) < 0)
		printf("Allocation problem \n");

	if (alloc_double_matrix(&K1H, 3, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&K2_inv, 3, 3, NULL) < 0)
		printf("Allocation problem \n");
	alloc_double_matrix(&C1_pose_struct, 4, 4, C1_pose);
	alloc_double_matrix(&C2_pose_struct, 4, 4, C2_pose);
	alloc_double_matrix(&K1_struct, 3, 3, K1);
	alloc_double_matrix(&K2_struct, 3, 3, K2);
	alloc_double_matrix(&n_struct, 1, 4, n);
	alloc_double_matrix(&H_struct, 3, 3, H);

	get_double_sub_matrix(&C1_pose_struct, &R_C1, 0, 0, 3, 3);
	get_double_sub_matrix(&C2_pose_struct, &R_C2, 0, 0, 3, 3);
	get_double_sub_matrix(&C1_pose_struct, &tC1, 3, 0, 1, 3);
	get_double_sub_matrix(&C2_pose_struct, &tC2, 3, 0, 1, 3);

	print_double_matrix(C1_pose_struct, "C1");
	print_double_matrix(C2_pose_struct, "C2");
	double_mat_inv(&C1_pose_struct, &C1_pose_struct_inv);
	print_double_matrix(C1_pose_struct_inv, "C1_inv");
	double_mat_product(&C2_pose_struct, &C1_pose_struct_inv, &C2C1_pose, 0, 0);
	print_double_matrix(C2C1_pose, "C2C1_pose");

	double intra = sqrt(
			pow(MAT_ELT_AT(C2C1_pose, 0, 3), 2)
					+ pow(MAT_ELT_AT(C2C1_pose, 1, 3), 2)
					+ pow(MAT_ELT_AT(C2C1_pose, 2, 3), 2));
	printf("%lf \n", intra);

	get_double_sub_matrix(&C2C1_pose, &R_C2C1, 0, 0, 3, 3);
	get_double_sub_matrix(&C2C1_pose, &tC2_C2C1, 3, 0, 1, 3);

	print_double_matrix(R_C2C1, "R_C2_C1");
	print_double_matrix(tC2_C2C1, "tC2_C2C1");

	print_double_matrix(R_C1, "RC1");
	print_double_matrix(n_struct, "n");
	get_double_sub_matrix(&n_struct, NULL, 0, 0, 1, 3);
	get_double_sub_matrix(&nC1, NULL, 0, 0, 1, 3);
	double_mat_product(&R_C1, &n_struct, &nC1, 0, 0);
	print_double_matrix(nC1, "nC1");
	print_double_matrix(tC1, "tC1");
	get_double_sub_matrix(&nC1, NULL, 0, 0, 1, 3);
	double_mat_product(&nC1, &tC1, &dC1_mat, 1, 0);
	print_double_matrix(dC1_mat, "dC1");
	double_mat_product(&tC2_C2C1, &nC1, &t_C2_C2C1_nC1t, 0, 1);
	print_double_matrix(t_C2_C2C1_nC1t, "t_C2_C2C1_nC1t");
	dC1 = -MAT_ELT_AT(dC1_mat, 0, 0);
	for (i = 0; i < t_C2_C2C1_nC1t.nbr; i++) {
		for (j = 0; j < t_C2_C2C1_nC1t.nbc; j++) {
			MAT_ELT_AT(t_C2_C2C1_nC1t, i , j) /= dC1;
		}
	}

	double_mat_sub(&R_C2C1, &t_C2_C2C1_nC1t, &H_struct);
	/*	double_mat_product(&K1_struct, &H_struct, &K1H, 0, 0);
	 double_mat_inv(&K2_struct, &K2_inv);
	 double_mat_product(&K1H, &K2_inv, &H_struct, 0, 0);*/
	print_double_matrix(H_struct, "H");

	free_double_matrix(&R_C1);
	free_double_matrix(&R_C2);
	free_double_matrix(&R_C2C1);
	free_double_matrix(&tC2_C2C1);
	free_double_matrix(&nC1);
	free_double_matrix(&tC1);
	free_double_matrix(&tC2);
	free_double_matrix(&dC1_mat);
	free_double_matrix(&t_C2_C2C1_nC1t);

}

//Function to compute the projection of a ground pixel in camera 1
/**
 * Camera
 */
int compute_ground_pixel_projection(double * uv_c1, double * uv_c2, double * n,
		double d, double * C1_matrix, double * C2_matrix) {
	// matrices are expressed as a camera |R t| a 4x4 matrix
	//                                    |0 1|
	double u = uv_c1[0];
	double v = uv_c1[1];
	struct double_matrix matA_struct, matA_inv_struct, matX_struct;
	struct double_matrix matB_struct, C2_mat_struct, C2_pixel;
	if (alloc_double_matrix(&matA_struct, 3, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&matA_inv_struct, 3, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&matB_struct, 1, 3, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&matX_struct, 1, 4, NULL) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&C2_mat_struct, 4, 4, C2_matrix) < 0)
		printf("Allocation problem \n");
	if (alloc_double_matrix(&C2_pixel, 1, 4, NULL) < 0)
		printf("Allocation problem \n");
	MAT_ELT_AT(matA_struct, 0, 0) = (C1_matrix[0 + (0 * 4)])
			- (C1_matrix[0 + (2 * 4)] * u);
	MAT_ELT_AT(matA_struct, 0, 1) = (C1_matrix[1 + (0 * 4)])
			- (C1_matrix[1 + (2 * 4)] * u);
	MAT_ELT_AT(matA_struct, 0, 2) = (C1_matrix[2 + (0 * 4)])
			- (C1_matrix[2 + (2 * 4)] * u);

	MAT_ELT_AT(matB_struct,0, 0) = (C1_matrix[3 + (2 * 4)] * u)
			- (C1_matrix[3 + (0 * 4)]);

	MAT_ELT_AT(matA_struct, 1, 0) = (C1_matrix[0 + (1 * 4)])
			- (C1_matrix[0 + (2 * 4)] * v);
	MAT_ELT_AT(matA_struct, 1, 1) = (C1_matrix[1 + (1 * 4)])
			- (C1_matrix[1 + (2 * 4)] * v);
	MAT_ELT_AT(matA_struct, 1, 2) = (C1_matrix[2 + (1 * 4)])
			- (C1_matrix[2 + (2 * 4)] * v);

	MAT_ELT_AT(matB_struct,0, 1) = (C1_matrix[3 + (2 * 4)] * v)
			- (C1_matrix[3 + (1 * 4)]);

	MAT_ELT_AT(matA_struct, 2, 0) = n[0];
	MAT_ELT_AT(matA_struct, 2, 1) = n[1];
	MAT_ELT_AT(matA_struct, 2, 2) = n[2];

	MAT_ELT_AT(matB_struct,0, 2) = -d;

	MAT_ELT_AT(matX_struct, 0, 3) = 1.;

	get_double_sub_matrix(&matX_struct, NULL, 0, 0, 1, 3);
	//print_double_matrix(matA_struct, "matA");

	//should compute A_inv directly ...

	double_mat_inv(&matA_struct, &matA_inv_struct);
	//print_double_matrix(matA_inv_struct, "matA_inv");
	//print_double_matrix(matB_struct, "matB");
	//if(double_mat_inv(&matA_struct, &matA_inv_struct) <0) return -1;
	if (double_mat_product(&matA_inv_struct, &matB_struct, &matX_struct, 0, 0)
			< 0)
		return -1;

	get_double_sub_matrix(&matX_struct, NULL, 0, 0, 1, 4);
	MAT_ELT_AT(matX_struct, 3, 0) = 1.0;
	//print_double_matrix(matX_struct, "matX");

	//print_double_matrix(C2_mat_struct, "matC2");
	double_mat_product(&C2_mat_struct, &matX_struct, &C2_pixel, 0, 0);
	MAT_ELT_AT(C2_pixel, 0, 0) = MAT_ELT_AT(C2_pixel, 0,
			0) / MAT_ELT_AT(C2_pixel, 2, 0);
	MAT_ELT_AT(C2_pixel, 1, 0) = MAT_ELT_AT(C2_pixel, 1,
			0) / MAT_ELT_AT(C2_pixel, 2, 0);
	MAT_ELT_AT(C2_pixel, 2, 0) = 1.;

	uv_c2[0] = MAT_ELT_AT(C2_pixel, 0, 0);
	uv_c2[1] = MAT_ELT_AT(C2_pixel, 1, 0);

	//print_double_matrix(C2_pixel, "matC2_pixel");

	free_double_matrix(&matA_struct);
	free_double_matrix(&matA_inv_struct);
	free_double_matrix(&matB_struct);
	free_double_matrix(&matX_struct);
	free_double_matrix(&C2_pixel);
	return 1;
}

void map_from_homography(char * img_in, unsigned int w_in, unsigned int h_in,
		char * img_out, unsigned int w_out, unsigned int h_out, double * H,
		double scale) {
	unsigned int u, v;
	for (u = 0; u < w_out; u++) {
		for (v = 0; v < h_out; v++) {

			//double xn = 0.997535, yn = 2.060746 ;
			double xn = (v - (h_out / 2.)) / scale;
			double yn = u / scale;

			double xnp = xn * H[0] + yn * H[1] + H[2];
			double ynp = xn * H[3] + yn * H[4] + H[5];
			double wnp = xn * H[6] + yn * H[7] + H[8];

			double xnp_norm = xnp / wnp;
			double ynp_norm = ynp / wnp;

			int hu = round(xnp_norm);
			int hv = round(ynp_norm);

			if (hu >= 0 && hv >= 0 && hu < w_in && hv < h_in) {
				unsigned char pixel_val = img_in[(hu) + ((hv) * w_in)];
				/*unsigned char pixel_val = ((hu/50)%2 ^ (hv)%2)*255;
				 parking_image->imageData[(hu) + ((hv) *  parking_image->width)] = 0 ;*/
				img_out[u + (v * w_out)] = pixel_val;
			} else {
				img_out[u + (v * w_out)] = 128;
			}
		}
	}
}

