#include "interpolate.hpp"

void compute_interpolation(float * x, float * y, float * params, int nb_params,
		int nb_samples) {
	int i, j;
	MatrixXf Rd(nb_samples, nb_params);
	Map<MatrixXf> rminusrd(y, nb_samples, 1);
	Map<MatrixXf> poly(params, nb_params, 1);

	for (i = 0; i < nb_samples; i++) {
		for (j = 0; j < nb_params; j++) {
			Rd(i, j) = pow(x[i], j);
		}
	}
	poly = (Rd.transpose() * Rd).llt().solve(Rd.transpose() * rminusrd);
}

