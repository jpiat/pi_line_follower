#include <iostream>
#include "resampling.hpp"
using namespace std;

void distort_radial(double * K, float u, float v, float * ud, float * vd,
		double * poly, unsigned int poly_size) {
	unsigned int i = 0;
	double xn = (u - K[6]) / K[0]; //vers plan image normalisé avec origine en axe principale et unité métrique
	double yn = (v - K[7]) / K[4];
	double r_square = xn * xn + yn * yn;
	double exp_x = r_square;
	double k = 1.0;
	for (i = 0; i < poly_size; i++) {
		k += exp_x * poly[i];
		exp_x *= r_square;
	}
	(*ud) = k * xn;
	(*vd) = k * yn;

	(*ud) = ((*ud)*K[0])+K[6] ;
	(*vd) = ((*vd)*K[4]) + K[7];

}

void undistort_radial(double * K, float ud, float vd, float * u, float * v,
		double * poly, unsigned int poly_size) {
	unsigned int i = 0;
	double xn = (ud - K[2]) / K[0]; //vers plan image normalisé avec origine en axe principale et unité métrique
	double yn = (vd - K[5]) / K[4];
	double r_square = xn * xn + yn * yn;
	double exp_x = r_square;
	double k = 1.0;
	for (i = 0; i < poly_size; i++) {
		k += exp_x * poly[i];
		exp_x *= r_square;
	}
	(*u) = k * xn;
	(*v) = k * yn;

	(*u) = ((*u)*K[0])+K[6] ;
	(*v) = ((*v)*K[4]) + K[7];
}

void calc_ct(double * world_to_cam, double * K, double * world_to_bot,
		double *Ct) {
	Map<MatrixXd> world_to_cam_eigen(world_to_cam, 3, 4);
	Map<MatrixXd> world_to_bot_eigen(world_to_bot, 4, 4);
	MatrixXd world_to_cam_in_bot(3, 4);
	Map<MatrixXd> K_eigen(K, 3, 3);
	Map<MatrixXd> Ct_eigen(Ct, 3, 4);
	world_to_cam_in_bot = world_to_cam_eigen * world_to_bot_eigen;
	Ct_eigen = K_eigen * world_to_cam_in_bot;
}

void bot_pos_in_cam_frame(double * world_to_cam, double * pos_in_world,
		double * pos_in_cam) {
	Map<MatrixXd> world_to_cam_eigen(world_to_cam, 3, 4);
	Map<MatrixXd> pos_eigen(pos_in_world, 4, 1);
	Map<MatrixXd> pos_in_cam_eigen(pos_in_cam, 3, 4);
	pos_in_cam_eigen = world_to_cam_eigen * pos_eigen;
}

void pixel_to_ground_plane(double * Ct, float u, float v, float * x,
		float * y) {

	Map<MatrixXd> Ct_eigen(Ct, 3, 4);

	double a1 = Ct_eigen(0, 0) - u * Ct_eigen(2, 0);
	double b1 = Ct_eigen(0, 1) - u * Ct_eigen(2, 1);
	double c1 = Ct_eigen(0, 2) - u * Ct_eigen(2, 2);

	double a2 = Ct_eigen(1, 0) - v * Ct_eigen(2, 0);
	double b2 = Ct_eigen(1, 1) - v * Ct_eigen(2, 1);
	double c2 = Ct_eigen(1, 2) - v * Ct_eigen(2, 2);

	double d1 = -(Ct_eigen(0, 3) - u * Ct_eigen(2, 3));
	double d2 = -(Ct_eigen(1, 3) - v * Ct_eigen(2, 3));

	double b3 = a2 * b1 - a1 * b2;
	double c3 = a2 * c1 - a1 * c2;
	double d3 = a2 * d1 - a1 * d2;

	(*y) = d3 / b3;
	(*x) = (d1 - ((*y) * b1)) / a1;

}

void ground_plane_to_pixel(double * Ct, double x, double y, float * u,
		float * v) {
	Map<MatrixXd> Ct_eigen(Ct, 3, 4);
	MatrixXd P_eigen(4, 1);
	MatrixXd p_eigen(3, 1);

	P_eigen(0, 0) = x;
	P_eigen(1, 0) = y;
	P_eigen(2, 0) = 0;
	P_eigen(3, 0) = 1;

	p_eigen = Ct_eigen * P_eigen;

	(*u) = p_eigen(0, 0) / p_eigen(2, 0);
	(*v) = p_eigen(1, 0) / p_eigen(2, 0);
}

double distort_radius(double r, double * poly, unsigned int nb_radial) {
	unsigned int i;
	double r_square = r * r;
	double r_acc = r_square;
	double k = 1.0 * r;
	for (i = 0; i < nb_radial; i++) {
		k += (r_acc * poly[i]) * r;
		r_acc *= r_acc;
	}
	return k;
}
