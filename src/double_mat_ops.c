/*
 * mat_ops.c
 *
 *  Created on: Jan 3, 2016
 *      Author: jpiat
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "double_mat_ops.h"

inline void double_scalar_vect_mac(double scalar, double * vec, double * res,
		int nbelt) {
	int i;
	if (scalar != 0) {
		for (i = 0; i < nbelt; i++)
			res[i] += scalar * vec[i];
	}
}

inline void double_vect_vect(double * vec1, double * vec2, double * res,
		int nbelt) {
	int i;
	(*res) = 0.;
	for (i = 0; i < nbelt; i++)
		(*res) += vec1[i] * vec2[i];
}

int double_mat_product(struct double_matrix * op1, struct double_matrix * op2,
		struct double_matrix * res, char op1_transpose, char op2_transpose) {
	int i, j;
	if ((op1->nbc != op2->nbr || res->nbc != op2->nbc || res->nbr != op1->nbr)
			&& (op1_transpose == 0) && (op2_transpose == 0))
		return -1;

	if ((op1->nbc != op2->nbc || res->nbc != op2->nbr || res->nbr != op1->nbr)
			&& (op1_transpose == 0) && (op2_transpose == 1))
		return -1;

	if ((op1->nbr != op2->nbr || res->nbc != op2->nbc || res->nbr != op1->nbc)
			&& (op1_transpose == 1) && (op2_transpose == 0))
		return -1;

	if ((op1->nbr != op2->nbc || res->nbc != op2->nbr || res->nbr != op1->nbc)
			&& (op1_transpose == 1) && (op2_transpose == 1))
		return -1;

	memset(res->data, 0, res->nbc * (res->nbr) * sizeof(double));
	if ((!op2_transpose && !op1_transpose)) {
		for (i = 0; i < op1->nbc; i++) {
			for (j = 0; j < op1->nbr; j++) {

				double_scalar_vect_mac(MAT_ELT_AT((*op1), j, i),
						MAT_GET_ROW((*op2), i), MAT_GET_ROW((*res), j),
						op2->nbc);

			}
		}
	} else if (!op2_transpose && op1_transpose) {
		for (i = 0; i < op1->nbr; i++) {
			for (j = 0; j < op1->nbc; j++) {
				double_scalar_vect_mac(MAT_ELT_AT((*op1), i, j),
						MAT_GET_ROW((*op2), i), MAT_GET_ROW((*res), j),
						op2->nbc);
			}
		}
	} else if (op2_transpose && !op1_transpose) {
		for (i = 0; i < op1->nbr; i++) {
			for (j = 0; j < op2->nbr; j++) {
				double_vect_vect(MAT_GET_ROW((*op2), j), MAT_GET_ROW((*op1), i),
						&(MAT_ELT_AT((*res), i, j)), op1->nbc);
			}
		}
	} else if (op2_transpose && op1_transpose) {
		for (i = 0; i < op2->nbr; i++) {
			for (j = 0; j < op2->nbc; j++) {
				double_scalar_vect_mac(MAT_ELT_AT((*op2), j, i),
						MAT_GET_ROW((*op1), i), MAT_GET_ROW((*res), j),
						op1->nbc);
			}
		}
		double_mat_transpose(res, NULL);
	}

	return 1;
}

int double_mat_transpose(struct double_matrix * op1, struct double_matrix * res) {
	struct double_matrix * transposed;
	int i, j;
	if (res == NULL) {
		transposed = malloc(sizeof(struct double_matrix));
		alloc_double_matrix(transposed, op1->nbr, op1->nbc, NULL);
	} else {
		transposed = res;
		transposed->nbc = op1->nbr;
		transposed->nbr = op1->nbc;
	}

	for (i = 0; i < op1->nbc; i++) {
		for (j = 0; j < op1->nbr; j++) {
			MAT_ELT_AT((*transposed), i, j) = MAT_ELT_AT((*op1), j, i);
		}
	}
	if (res == NULL) {
		memcpy(op1->data, transposed->data,
				op1->nbc * op1->nbr * sizeof(double));
		op1->nbr = transposed->nbr;
		op1->nbc = transposed->nbc;
		free(transposed->data);
	}
	return 1;
}

int double_mat_trace(struct double_matrix * op1, double * trace) {
	int i;
	if (op1->nbc != op1->nbr)
		return -1; //not square
	(*trace) = 0.;
	for (i = 0; i < op1->nbc; i++) {
		(*trace) += MAT_ELT_AT((*op1), i, i);
	}

	return 1;
}

int double_matrix_minor(struct double_matrix * src, struct double_matrix * dest,
		int row, int col) {
	int i, j;
	// indicate which col and row is being copied to dest
	int colCount = 0, rowCount = 0;

	for (i = 0; i < src->nbc; i++) {
		if (i != row) {
			colCount = 0;
			for (j = 0; j < src->nbr; j++) {
				// when j is not the element
				if (j != col) {
					MAT_ELT_AT((*dest), rowCount, colCount) = MAT_ELT_AT(
							(*src), i, j);
					colCount++;
				}
			}
			rowCount++;
		}
	}

	return 1;
}

// Calculate the determinant recursively.
double double_matrix_calc_ndp_det(struct double_matrix * mat) {
	int i;
	// order must be >= 0
	// stop the recursion when matrix is a single element
	if (mat->nbc == 1)
		return MAT_ELT_AT((*mat), 0, 0);

	// the determinant value
	double det = 0;

	// allocate the cofactor matrix
	struct double_matrix minor;
	alloc_double_matrix(&minor, mat->nbc - 1, mat->nbr - 1, NULL);
	for (i = 0; i < mat->nbc; i++) {
		// get minor of element (0,i)
		double_matrix_minor(mat, &minor, 0, i);
		// the recusion is here!

		det += (i % 2 == 1 ? -1.0 : 1.0) * MAT_ELT_AT((*mat), 0, i)
				* double_matrix_calc_ndp_det(&minor);
		//det += pow( -1.0, i ) * mat[0][i] * CalcDeterminant( minor,order-1 );
	}
	free_double_matrix(&minor);
	return det;
}

// matrix inversioon
// the result is put in Y
int double_matrix_ndp_inversion(struct double_matrix *A,
		struct double_matrix * A_inv) {
	int i, j;
	// get the determinant of a
	double det = 1.0 / double_matrix_calc_ndp_det(A);
	if (det == 0 || A->nbc != A->nbr)
		return -1;
	struct double_matrix minor;
	alloc_double_matrix(&minor, A->nbc - 1, A->nbr - 1, NULL);

	for (j = 0; j < A->nbr; j++) {
		for (i = 0; i < A->nbc; i++) {
			// get the co-factor (matrix) of A(j,i)
			double_matrix_minor(A, &minor, j, i);
			MAT_ELT_AT((*A_inv), i, j) = det
					* double_matrix_calc_ndp_det(&minor);
			if ((i + j) % 2 == 1)
				MAT_ELT_AT((*A_inv), i,j) = -(MAT_ELT_AT((*A_inv), i, j));
		}
	}
	return 1;

}

//Following is adaptation of http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt
int double_choldc1(struct double_matrix * a, double * p);
/* -----------------------------------------------
 Cholesky decomposition.

 input    A  Symmetric positive def. matrix
 output   a  lower deomposed matrix
 uses        choldc1(MAT,VEC)
 ----------------------------------------------- */
int double_choldc(struct double_matrix *A, struct double_matrix * a) {
	int i, j;
	double * p = malloc(sizeof(double) * (A->nbc));
	copy_double_matrix(A, a, 1);
	if (double_choldc1(a, p) < 0) {
		free(p);
		return -1;
	}
	for (i = 0; i < a->nbr; i++) {
		MAT_ELT_AT((*a), i, i) = p[i];
		for (j = i + 1; j < a->nbc; j++) {
			MAT_ELT_AT((*a), i, j) = 0;
		}
	}
	free(p);
	return 1;
}

/* -----------------------------------------------------
 Inverse of Cholesky decomposition.
 input    A  Symmetric positive def. matrix
 output   a  inverse of lower deomposed matrix
 uses        choldc1(MAT,VEC)
 ----------------------------------------------------- */
int double_choldcsl(struct double_matrix A, struct double_matrix * a) {
	int i, j, k;
	double sum;
	double * p = malloc(sizeof(double) * A.nbc);
	copy_double_matrix(&A, a, 0);

	if (double_choldc1(a, p) < 0) {
		free(p);
		return -1;
	}
	for (i = 0; i < a->nbr; i++) {
		MAT_ELT_AT((*a), i, i) = 1. / p[i];
		for (j = i + 1; j < a->nbc; j++) {
			sum = 0;
			for (k = i; k < j; k++) {
				sum -= MAT_ELT_AT((*a), j, k) * MAT_ELT_AT((*a), k, i);
			}
			MAT_ELT_AT((*a), j, i) = sum / p[j];
		}
	}
	free(p);
	return 1;
}

/* -----------------------------------------------------------------------------
 Computation of Determinant of the matrix using Cholesky decomposition

 input    a  Symmetric positive def. matrix
 return      det(a)
 uses        choldc(MAT,MAT)
 ------------------------------------------------------------------------------ */
double double_mat_det(struct double_matrix * a) {
	struct double_matrix c;
	double d = 1;
	int i;
	if (double_choldc(a, &c) < 0)
		return 0.0;
	for (i = 0; i < a->nbc; i++)
		d *= MAT_ELT_AT(c, i, i);
	return d * d;
}

/* ---------------------------------------------------
 Matrix inverse using Cholesky decomposition

 input	  A  Symmetric positive def. matrix
 output   a  inverse of A
 uses        choldc1(MAT, VEC)
 --------------------------------------------------- */
int double_mat_inv(struct double_matrix * A, struct double_matrix *a) {
	int i, j, k;
	if (double_choldcsl((*A), a) < 0)
		return double_matrix_ndp_inversion(A, a);
	for (i = 0; i < A->nbc; i++) {
		for (j = i + 1; j < A->nbc; j++) {
			MAT_ELT_AT((*a),i, j) = 0.0;
		}
	}
	for (i = 0; i < a->nbr; i++) {
		MAT_ELT_AT((*a),i, i) *= MAT_ELT_AT((*a), i, i);
		for (k = i + 1; k < a->nbc; k++) {
			MAT_ELT_AT((*a),i, i) += MAT_ELT_AT((*a), k, i
			) * MAT_ELT_AT((*a), k, i);
		}
		for (j = i + 1; j < a->nbc; j++) {
			for (k = j; k < a->nbc; k++) {
				MAT_ELT_AT((*a), i, j) += MAT_ELT_AT((*a), k,
						i) * MAT_ELT_AT((*a), k, j);
			}
		}
	}
	for (i = 0; i < a->nbr; i++) {
		for (j = 0; j < i; j++) {
			MAT_ELT_AT((*a), i, j) = MAT_ELT_AT((*a), j, i);
		}
	}
	return 1;
}

int double_choldc1(struct double_matrix * a, double * p) {
	int i, j, k;
	double sum;
	for (i = 0; i < a->nbr; i++) {
		for (j = i; j < a->nbc; j++) {
			sum = MAT_ELT_AT((*a), i, j);
			for (k = i - 1; k >= 0; k--) {
				sum -= MAT_ELT_AT((*a), i, k) * MAT_ELT_AT((*a), j, k);
			}
			if (i == j) {
				if (sum <= 0) {
					return -1;
				}
				p[i] = sqrt(sum);
			} else {
				MAT_ELT_AT((*a),j, i) = sum / p[i];
			}
		}
	}
	return 1;
}

int double_mat_vect_product(struct double_matrix * op1, double * vec,
		double * res) {
	double * rest;
	int i;
	if (vec == res) {
		rest = malloc(op1->nbr * sizeof(double));
	} else {
		rest = res;
	}
	memset(rest, 0, op1->nbr * sizeof(double));
	for (i = 0; i < op1->nbr; i++) {
		double_vect_vect(MAT_GET_ROW((*op1), i), vec, &(rest[i]), op1->nbc);
	}
	if (vec == res) {
		memcpy(res, rest, op1->nbr * sizeof(double));
	}
	return 1;
}

void double_vec_add(double * op1, double * op2, double * res, unsigned int size) {
	int i;
	for (i = 0; i < size; i++)
		res[i] = op1[i] + op2[i];
}
void double_vec_sub(double * op1, double * op2, double * res, unsigned int size) {
	int i;
	for (i = 0; i < size; i++)
		res[i] = op1[i] - op2[i];
}
int double_mat_add(struct double_matrix * op1, struct double_matrix * op2,
		struct double_matrix * res) {
	int i;
	if (op1->nbr != op2->nbr || op1->nbc != op2->nbc)
		return -1;
	for (i = 0; i < op1->nbr; i++) {
		double_vec_add(MAT_GET_ROW((*op1), i), MAT_GET_ROW((*op2), i),
				MAT_GET_ROW((*res), i), op1->nbc);
	}
	return 1;
}
int double_mat_sub(struct double_matrix * op1, struct double_matrix * op2,
		struct double_matrix * res) {
	int i;
	if (op1->nbr != op2->nbr || op1->nbc != op2->nbc)
		return -1;
	for (i = 0; i < op1->nbr; i++) {
		double_vec_sub(MAT_GET_ROW((*op1), i), MAT_GET_ROW((*op2), i),
				MAT_GET_ROW((*res), i), op1->nbc);
	}
	return 1;
}

int double_mat_eye_sub(struct double_matrix * op1, struct double_matrix * res) {
	int i, j;
	for (i = 0; i < op1->nbc; i++) {
		for (j = 0; j < op1->nbr; j++) {
			if (i == j) {
				MAT_ELT_AT((*res), j, i) = 1. - MAT_ELT_AT((*op1), j, i);
			} else {
				MAT_ELT_AT((*res), j, i) = -MAT_ELT_AT((*op1), j, i);
			}
		}
	}
	return 1;
}
