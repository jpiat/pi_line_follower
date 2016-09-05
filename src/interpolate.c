#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <float.h>

#define MAT_ELT(m, r, c, nb_col) (m[r*nb_col+c])

#define SIGN(a,b) ((b) > 0.0 ? fabs(a) : -fabs(a))

static float maxarg1, maxarg2;
#define FMAX(a,b) (maxarg1 = (a),maxarg2 = (b),(maxarg1) > (maxarg2) ? (maxarg1) : (maxarg2))

static int iminarg1, iminarg2;
#define IMIN(a,b) (iminarg1 = (a),iminarg2 = (b),(iminarg1 < (iminarg2) ? (iminarg1) : iminarg2))

static float sqrarg;
#define SQR(a) ((sqrarg = (a)) == 0.0 ? 0.0 : sqrarg * sqrarg)

int svdcmp(float *a, int nRows, int nCols, float *w, float *v);

// prints an arbitrary size matrix to the standard output
void printMatrix(float *a, int rows, int cols);
void printMatrix(float *a, int rows, int cols) {
#ifdef DEBUG
	int i, j;
	printf("[");
	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			printf("%.8lf ", MAT_ELT(a, i, j, cols));
		}
		printf(";\n");
	}
	printf(" ]");
	printf("\n");
#endif
}

// calculates sqrt( a^2 + b^2 ) with decent precision
float pythag(float a, float b);
float pythag(float a, float b) {
	float absa, absb;

	absa = fabs(a);
	absb = fabs(b);

	if (absa > absb)
		return (absa * sqrt(1.0 + SQR(absb / absa)));
	else
		return (absb == 0.0 ? 0.0 : absb * sqrt(1.0 + SQR(absa / absb)));
}

/*
 Modified from Numerical Recipes in C
 Given a matrix a[nRows][nCols], svdcmp() computes its singular value
 decomposition, A = U * W * Vt.  A is replaced by U when svdcmp
 returns.  The diagonal matrix W is output as a vector w[nCols].
 V (not V transpose) is output as the matrix V[nCols][nCols].
 */
void reciprocal_vec(float * w, int nCols) {
	int i;
	float t = DBL_EPSILON * nCols * w[0];
	for (i = 0; i < nCols; i++) {
		float abs_wi = w[i] < 0 ? -w[i] : w[i];
		if (abs_wi >= t) { //should be computed from machine epsilon
			w[i] = 1. / w[i];
		} else {
			w[i] = 0.;
		}
	}
}

void transpose(float * w, float * wt, int nCols, int nbRows) {
	int i, j;
	for (i = 0; i < nbRows; i++) {
		for (j = 0; j < nCols; j++) {
			MAT_ELT(wt, j, i, nbRows) = MAT_ELT(w, i, j, nCols);
		}
	}

}

int svdcmp(float *a, int nRows, int nCols, float *w, float *v) {
	int flag, i, its, j, jj, k, l, nm;
	float anorm, c, f, g, h, s, scale, x, y, z, *rv1;

	rv1 = malloc(sizeof(float) * nCols);
	if (rv1 == NULL) {
		printf("svdcmp(): Unable to allocate vector\n");
		return (-1);
	}

	g = scale = anorm = 0.0;
	for (i = 0; i < nCols; i++) {
		l = i + 1;
		rv1[i] = scale * g;
		g = s = scale = 0.0;
		if (i < nRows) {
			for (k = i; k < nRows; k++)
				scale += fabs(MAT_ELT(a, k, i, nCols));
			if (scale) {
				for (k = i; k < nRows; k++) {
					MAT_ELT(a, k, i, nCols) /= scale;
					s += MAT_ELT(a, k, i, nCols) * MAT_ELT(a, k, i, nCols);
				}
				f = MAT_ELT(a, i, i, nCols);
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				MAT_ELT(a, i, i, nCols) = f - g;
				for (j = l; j < nCols; j++) {
					for (s = 0.0, k = i; k < nRows; k++)
						s += MAT_ELT(a, k, i, nCols) * MAT_ELT(a, k, j, nCols);
					f = s / h;
					for (k = i; k < nRows; k++)
						MAT_ELT(a, k, j, nCols) += f * MAT_ELT(a, k, i, nCols);
				}
				for (k = i; k < nRows; k++)
					MAT_ELT(a, k, i, nCols) *= scale;
			}
		}
		w[i] = scale * g;
		g = s = scale = 0.0;
		if (i < nRows && i != nCols - 1) {
			for (k = l; k < nCols; k++)
				scale += fabs(MAT_ELT(a, i, k, nCols));
			if (scale) {
				for (k = l; k < nCols; k++) {
					MAT_ELT(a, i, k, nCols) /= scale;
					s += MAT_ELT(a, i, k, nCols) * MAT_ELT(a, i, k, nCols);
				}
				f = MAT_ELT(a, i, l, nCols);
				g = -SIGN(sqrt(s), f);
				h = f * g - s;
				MAT_ELT(a,i, l, nCols) = f - g;
				for (k = l; k < nCols; k++)
					rv1[k] = MAT_ELT(a, i, k, nCols) / h;
				for (j = l; j < nRows; j++) {
					for (s = 0.0, k = l; k < nCols; k++)
						s += MAT_ELT(a,j, k, nCols) * MAT_ELT(a, i, k, nCols);
					for (k = l; k < nCols; k++)
						MAT_ELT(a,j, k, nCols) += s * rv1[k];
				}
				for (k = l; k < nCols; k++)
					MAT_ELT(a,i, k, nCols) *= scale;
			}
		}
		anorm = FMAX(anorm, (fabs(w[i]) + fabs(rv1[i])));

		/*printf(".");
		fflush(stdout);*/
	}

	for (i = nCols - 1; i >= 0; i--) {
		if (i < nCols - 1) {
			if (g) {
				for (j = l; j < nCols; j++)
					MAT_ELT(v, j, i, nCols) = (MAT_ELT(a, i, j, nCols)
							/ MAT_ELT(a, i, l, nCols)) / g;
				for (j = l; j < nCols; j++) {
					for (s = 0.0, k = l; k < nCols; k++)
						s += MAT_ELT(a, i, k, nCols) * MAT_ELT(v, k, j, nCols);
					for (k = l; k < nCols; k++)
						MAT_ELT(v, k, j, nCols) += s * MAT_ELT(v, k, i, nCols);
				}
			}
			for (j = l; j < nCols; j++)
				MAT_ELT(v, i, j, nCols) = MAT_ELT(v, j, i, nCols) = 0.0;
		}
		MAT_ELT(v, i, i, nCols) = 1.0;
		g = rv1[i];
		l = i;
		/*printf(".");
		fflush(stdout);*/
	}

	for (i = IMIN(nRows,nCols) - 1; i >= 0; i--) {
		l = i + 1;
		g = w[i];
		for (j = l; j < nCols; j++)
			MAT_ELT(a, i, j, nCols) = 0.0;
		if (g) {
			g = 1.0 / g;
			for (j = l; j < nCols; j++) {
				for (s = 0.0, k = l; k < nRows; k++)
					s += MAT_ELT(a, k, i, nCols) * MAT_ELT(a, k, j, nCols);
				f = (s / MAT_ELT(a, i, i, nCols)) * g;
				for (k = i; k < nRows; k++)
					MAT_ELT(a, k, j, nCols) += f * MAT_ELT(a, k, i, nCols);
			}
			for (j = i; j < nRows; j++)
				MAT_ELT(a, j, i, nCols) *= g;
		} else
			for (j = i; j < nRows; j++)
				MAT_ELT(a, j, i, nCols) = 0.0;
		++MAT_ELT(a, i, i, nCols);
		/*printf(".");
		fflush(stdout);*/
	}

	for (k = nCols - 1; k >= 0; k--) {
		for (its = 0; its < 30; its++) {
			flag = 1;
			for (l = k; l >= 0; l--) {
				nm = l - 1;
				if ((fabs(rv1[l]) + anorm) == anorm) {
					flag = 0;
					break;
				}
				if ((fabs(w[nm]) + anorm) == anorm)
					break;
			}
			if (flag) {
				c = 0.0;
				s = 1.0;
				for (i = l; i <= k; i++) {
					f = s * rv1[i];
					rv1[i] = c * rv1[i];
					if ((fabs(f) + anorm) == anorm)
						break;
					g = w[i];
					h = pythag(f, g);
					w[i] = h;
					h = 1.0 / h;
					c = g * h;
					s = -f * h;
					for (j = 0; j < nRows; j++) {
						y = MAT_ELT(a, j, nm, nCols);
						z = MAT_ELT(a, j, i, nCols);
						MAT_ELT(a, j, nm, nCols) = y * c + z * s;
						MAT_ELT(a, j, i, nCols) = z * c - y * s;
					}
				}
			}
			z = w[k];
			if (l == k) {
				if (z < 0.0) {
					w[k] = -z;
					for (j = 0; j < nCols; j++)
						MAT_ELT(v, j, k , nCols) = -MAT_ELT(v, j, k, nCols);
				}
				break;
			}
			if (its == 29)
				printf("no convergence in 30 svdcmp iterations\n");
			x = w[l];
			nm = k - 1;
			y = w[nm];
			g = rv1[nm];
			h = rv1[k];
			f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
			g = pythag(f, 1.0);
			f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
			c = s = 1.0;
			for (j = l; j <= nm; j++) {
				i = j + 1;
				g = rv1[i];
				y = w[i];
				h = s * g;
				g = c * g;
				z = pythag(f, h);
				rv1[j] = z;
				c = f / z;
				s = h / z;
				f = x * c + g * s;
				g = g * c - x * s;
				h = y * s;
				y *= c;
				for (jj = 0; jj < nCols; jj++) {
					x = MAT_ELT(v, jj, j, nCols);
					z = MAT_ELT(v, jj, i, nCols);
					MAT_ELT(v, jj, j, nCols) = x * c + z * s;
					MAT_ELT(v, jj, i, nCols) = z * c - x * s;
				}
				z = pythag(f, h);
				w[j] = z;
				if (z) {
					z = 1.0 / z;
					c = f * z;
					s = h * z;
				}
				f = c * g + s * y;
				x = c * y - s * g;
				for (jj = 0; jj < nRows; jj++) {
					y = MAT_ELT(a, jj, j, nCols);
					z = MAT_ELT(a, jj, i, nCols);
					MAT_ELT(a, jj, j, nCols) = y * c + z * s;
					MAT_ELT(a, jj, i, nCols) = z * c - y * s;
				}
			}
			rv1[l] = 0.0;
			rv1[k] = f;
			w[k] = x;
		}
		/*printf(".");
		fflush(stdout);*/
	}
	//printf("\n");

	free(rv1);

	return (0);
}

float distort(float r, float * poly, int nb_radial) {
	int i;
	float r_square = r * r;
	float r_acc = r_square;
	float k = 1.0 * r;
	for (i = 0; i < nb_radial; i++) {
		k += (r_acc * poly[i]) * r;
		r_acc *= r_acc;
	}
	return k;
}

int float_mat_product(float * op1, float * op2, float * res, int op1_nbc,
		int op1_nbr, int op2_nbc, int op2_nbr) {
	int i, j, k;
	memset(res, 0, op2_nbc * (op1_nbr) * sizeof(float));
	for (i = 0; i < op1_nbr; i++) {
		for (j = 0; j < op2_nbc; j++) {
			MAT_ELT(res, i, j, op2_nbc) = 0.;
			for (k = 0; k < op1_nbc; k++) {
				MAT_ELT(res, i, j, op2_nbc) += MAT_ELT(op1, i, k,
						op1_nbc) * MAT_ELT(op2, k, j, op2_nbc);
			}
		}
	}
	return 1;
}

void pseudo_inverse(float * a, float * a_inv, int nCols, int nRows) {
	int i, j, k;
	float * a_copy = malloc(nCols * nRows * sizeof(float));
	float * Ut = malloc(nCols * nRows * sizeof(float));
	memcpy(a_copy, a, nCols * nRows * sizeof(float));
	float * v = malloc(nCols * nCols * sizeof(float));
	float * vw = malloc(nCols * nCols * sizeof(float));
	memset(v, 0, nCols * nCols * sizeof(float));
	float * w = malloc(nCols * sizeof(float));
	memset(w, 0, nCols * sizeof(float));
	//printMatrix(a_copy, nCols, nRows);
	svdcmp(a_copy, nRows, nCols, w, v);
	//printMatrix(w, nCols, 1);
	//printMatrix(v, nCols, nCols);
	//printMatrix(a_copy,nRows, nCols);
	reciprocal_vec(w, nCols);
	//printMatrix(w, nCols, 1);
	//vw = V*w
	for (i = 0; i < nCols; i++) {
		for (j = 0; j < nCols; j++) {
			MAT_ELT(vw, i, j, nCols) = w[j] * MAT_ELT(v, i, j, nCols);
		}
	}
	printMatrix(vw, nCols, nCols);
	//a_inv = vW*Ut
	transpose(a_copy, Ut, nCols, nRows);
	float_mat_product(vw, Ut, a_inv, nCols, nCols, nRows, nCols);
	free(vw);
	free(a_copy);
	free(v);
	free(w);
	free(Ut);
	//should have inverse in a_inv
}

void compute_interpolation(float * x, float * y , float * params, int nb_params, int nb_samples) {
	int i, j;
	float * Rd = malloc(nb_params * nb_samples * sizeof(float));
	float * Rd_inv = malloc(nb_params * nb_samples * sizeof(float));
	float * rminusrd = malloc(nb_samples * sizeof(float));
	float * fct_params;
	if (params == NULL) {
		fct_params = malloc(nb_params * sizeof(float));
	} else {
		fct_params = params;
	}

	for (i = 0; i < nb_samples; i++) {
		for (j = 0; j < nb_params; j++) {
			Rd[(i * nb_params) + j] = pow(x[i], j);
		}
		rminusrd[i] = y[i];
	}
	//compute pseudo inverse of RdTRd
	/*printMatrix(Rd, nb_samples, nb_params);
	printMatrix(rminusrd, nb_samples, 1);*/
	pseudo_inverse(Rd, Rd_inv, nb_params, nb_samples);
	//printMatrix(Rd_inv, nb_undistort_params, nb_samples);
	float_mat_product(Rd_inv, rminusrd, params, nb_samples,
			nb_params, 1, nb_samples);
	//printMatrix(params, nb_params, 1);
	float max_error = 0.0;
	float sum_error = 0.0;
	for (i = 0; i < nb_samples; i++) {
		float y_inter = 0.;
		float error ;
		for(j = 0 ; j < nb_params ; j ++){
			y_inter += params[j] * pow(x[i], j) ;
		}
		error = abs(y_inter - y[i]);
		if(error > max_error){
			max_error = error ;
		}
		sum_error += error ;
	}
	/*printf("Error max = %lf \n", max_error);
	printf("Error mean = %lf \n", sum_error / nb_samples);*/
	/*printf("Error max pixels = %lf \n", max_error * k[2]);
	float mean_error = sum_error / nb_samples;
	float dev_err = 0.0;
	for (i = 0; i < nb_samples; i++) {
		float ri = ((i + 1) * step);
		float distorded = distort(((i + 1) * step), direct_distort,
				nb_distort_params);
		float undistorded = distort(distorded, inverse_distort,
				nb_undistort_params);
		float error = ri - undistorded;
		dev_err += pow(error - mean_error, 2);

	}
	dev_err = dev_err / nb_samples ;
	printf("std_dev = %lf \n", sqrt(dev_err));


	//printMatrix(inverse_distort, nb_undistort_params, 1);
//multiply by RdT
	return inverse_distort;*/
}

