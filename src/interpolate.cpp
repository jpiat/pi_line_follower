#include "interpolate.hpp"

void compute_interpolation(float * x, float * y, float * params, int nb_params,
		int nb_samples) {
	int i, j;
	MatrixXf Rd(nb_samples, nb_params);
	MatrixXf rminusrd(nb_samples, 1);
	MatrixXf poly(nb_params, 1);

	for (i = 0; i < nb_samples; i++) {
		for (j = 0; j < nb_params; j++) {
			Rd(i, j) = pow(x[i], j);
		}
		rminusrd(i, 0) = y[i];
	}
	poly = (Rd.transpose() * Rd).llt().solve(Rd.transpose() * rminusrd);
	for (j = 0; j < nb_params; j++) {
		params[j] = poly(j);
	}
}

