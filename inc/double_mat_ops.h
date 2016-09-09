#include "mat_types.h"

#ifndef DOUBLE_MAT_OPS_H
#define DOUBLE_MAT_OPS_H

inline void double_scalar_vect_mac(double scalar, double * vec, double * res,
		int nbelt);
inline void double_vect_vect(double * vec1, double * vec2, double * res, int nbelt);
int double_mat_product(struct double_matrix * op1, struct double_matrix * op2, struct double_matrix * res, char op1_transpose, char op2_transpose);
int double_mat_vect_product(struct double_matrix * op1, double * vec, double * res);
void double_vec_add(double * op1, double * op2, double * res, unsigned int size);
void double_vec_sub(double * op1, double * op2, double * res, unsigned int size);
int double_mat_add(struct double_matrix * op1, struct double_matrix * op2,
		struct double_matrix * res);
int double_mat_sub(struct double_matrix * op1, struct double_matrix * op2,
		struct double_matrix * res);


int double_mat_transpose(struct double_matrix * op1, struct double_matrix * res);
double double_mat_det(struct double_matrix * op1);
int double_mat_inv(struct double_matrix * A, struct double_matrix *a);
int double_mat_eye_sub(struct double_matrix * op1, struct double_matrix * res);

#endif
