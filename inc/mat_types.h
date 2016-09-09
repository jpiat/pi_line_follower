/*
 * mat_types.h
 *
 *  Created on: Jan 3, 2016
 *      Author: jpiat
 */

#include <stddef.h>

#ifndef MAT_TYPES_H
#define MAT_TYPES_H

#define MAT_PTR_ELT_AT(m, row, col) (m->data[(m->origin_r*m->line_step) + (row)*m->line_step + m->origin_c + (col) ])
#define MAT_ELT_AT(m, row, col) (m.data[(m.origin_r*m.line_step) + (row)*m.line_step + m.origin_c + (col) ])
#define MAT_GET_ROW(m, row) (&(m.data[m.origin_r*m.line_step + (row)*m.line_step + m.origin_c]))

#define SYMMETRIC_MASK 0x01

struct float_matrix {
	unsigned int nbr;
	unsigned int nbc;
	unsigned int line_step;
	unsigned int origin_c;
	unsigned int origin_r;
	unsigned int alloc_size;
	unsigned char matrix_properties;
	float * data;
};

int alloc_float_matrix(struct float_matrix * mat, int nbc, int nbl,
		void * buffer);
void free_float_matrix(struct float_matrix * mat);
void zeros_float_matrix(struct float_matrix * mat);
void print_float_matrix(struct float_matrix mat, char * name);
void print_float_vec(float * vec, unsigned int size, char * name);
void identity_float_matrix(struct float_matrix * mat);
void random_float_matrix(struct float_matrix * mat);
void get_float_sub_matrix(struct float_matrix * op, struct float_matrix * sub,
		unsigned int origin_nbc, unsigned int origin_nbr, unsigned int nbc,
		unsigned int nbr);
void copy_float_matrix(struct float_matrix * src, struct float_matrix * dest,
		char alloc);


struct double_matrix {
	unsigned int nbr;
	unsigned int nbc;
	unsigned int line_step;
	unsigned int origin_c;
	unsigned int origin_r;
	unsigned int alloc_size;
	unsigned int matrix_properties;
	double * data;
};

int alloc_double_matrix(struct double_matrix * mat, int nbc, int nbl,
		void * buffer);
void free_double_matrix(struct double_matrix * mat);
void zeros_double_matrix(struct double_matrix * mat);
void print_double_matrix(struct double_matrix mat, char * name);
void print_double_vec(double * vec, unsigned int size, char * name);
void identity_double_matrix(struct double_matrix * mat);
void random_double_matrix(struct double_matrix * mat);
void get_double_sub_matrix(struct double_matrix * op, struct double_matrix * sub,
		unsigned int origin_nbc, unsigned int origin_nbr, unsigned int nbc,
		unsigned int nbr);
void copy_double_matrix(struct double_matrix * src, struct double_matrix * dest,
		char alloc);

#endif
