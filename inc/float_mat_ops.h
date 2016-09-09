#include "mat_types.h"

#ifndef FLOAT_MAT_OPS_H
#define FLOAT_MAT_OPS_H

inline void float_scalar_vect_mac(float scalar, float * vec, float * res,
		int nbelt);

inline void float_vect_vect(float * vec1, float * vec2, float * res, int nbelt);

int float_mat_product(struct float_matrix * op1, struct float_matrix * op2, struct float_matrix * res, char op1_transpose, char op2_transpose);
int float_mat_vect_product(struct float_matrix * op1, float * vec, float * res);
void float_vec_add(float * op1, float * op2, float * res, unsigned int size);
void float_vec_sub(float * op1, float * op2, float * res, unsigned int size);
int float_mat_add(struct float_matrix * op1, struct float_matrix * op2,
		struct float_matrix * res);
int float_mat_sub(struct float_matrix * op1, struct float_matrix * op2,
		struct float_matrix * res);

int float_mat_transpose(struct float_matrix * op1, struct float_matrix * res);
float float_mat_det(struct float_matrix * op1);
int float_mat_inv(struct float_matrix * A, struct float_matrix *a);
int float_mat_eye_sub(struct float_matrix * op1, struct float_matrix * res);


#endif
