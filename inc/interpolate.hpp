
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace Eigen ;
#ifndef INTERPOLATE_H
#define INTERPOLATE_H

void compute_interpolation(float * x, float * y , float * params, int nb_params, int nb_samples);

#endif
