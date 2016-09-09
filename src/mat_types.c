#include "mat_types.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int alloc_float_matrix(struct float_matrix * mat, int nbc, int nbr,
		void * buffer) {
	mat->nbc = nbc;
	mat->nbr = nbr;
	mat->line_step = nbc;
	if (buffer == NULL) {
		buffer = malloc(nbc * nbr * sizeof(float));
	}
	if (buffer == NULL)
		return -1;
	mat->alloc_size = nbc * nbr;
	mat->data = (float *) buffer;
	mat->origin_r = 0;
	mat->origin_c = 0;
	memset(buffer, 0, mat->alloc_size * sizeof(float));
	return mat->alloc_size;
}

void free_float_matrix(struct float_matrix * mat) {
	if (mat->data != NULL) {
		free(mat->data);
	}
	mat->data = NULL;
}

void zeros_float_matrix(struct float_matrix * mat) {
	memset(mat->data, 0, mat->alloc_size * sizeof(float));
}

void get_float_sub_matrix(struct float_matrix * op, struct float_matrix * sub,
		unsigned int origin_nbc, unsigned int origin_nbr, unsigned int nbc,
		unsigned int nbr) {
	if (sub == NULL) {
		op->origin_r = origin_nbr;
		op->origin_c = origin_nbc;
		op->nbc = nbc;
		op->nbr = nbr;
	} else {
		int i;
		sub->origin_r = 0;
		sub->origin_c = 0;
		sub->nbc = nbc;
		sub->nbr = nbr;
		sub->line_step = 2;
		if (sub->data == NULL) {
			sub->data = malloc(nbc * nbr * sizeof(float));
			sub->alloc_size = nbc * nbr;
		}
		for (i = 0; i < nbr; i++) {
			memcpy(MAT_GET_ROW((*sub), i),
					&(MAT_ELT_AT((*op), origin_nbr+ 1, origin_nbc)),
					nbc * sizeof(float));
		}
	}
}

void identity_float_matrix(struct float_matrix * mat) {
	int i;
	for (i = 0; i < mat->nbc; i++)
		MAT_ELT_AT((*mat), i, i) = 1.0;
}

void random_float_matrix(struct float_matrix * mat) {
	int i, j;
	for (i = 0; i < mat->nbr; i++) {
		for (j = 0; j < mat->nbc; j++)
			MAT_ELT_AT((*mat), i, j) = (float) rand() / ((float) RAND_MAX);
	}
}

void print_float_matrix(struct float_matrix mat, char * name) {
	int i, j;
	printf("%s = [\n", name);
	for (i = 0; i < mat.nbr; i++) {
		for (j = 0; j < mat.nbc; j++) {
			printf("%f\t", MAT_ELT_AT(mat, i, j));
		}
		printf(";\n");
	}
	printf("]\n");
}

void print_float_vec(float * vec, unsigned int size, char * name) {
	int j;
	printf("%s = [", name);
	for (j = 0; j < size; j++) {
		printf("%f\t", vec[j]);
	}
	printf("]\n");
}

void copy_float_matrix(struct float_matrix * src, struct float_matrix * dest,
		char alloc) {
	int i;
	if (alloc) {
		alloc_float_matrix(dest, src->nbc, src->nbr, NULL);
	} else {
		alloc_float_matrix(dest, src->nbc, src->nbr, dest->data);
	}
	for (i = 0; i < src->nbr; i++) {
		memcpy(MAT_GET_ROW((*dest), i), MAT_GET_ROW((*src), i),
				src->nbc * sizeof(float));
	}
}





int alloc_double_matrix(struct double_matrix * mat, int nbc, int nbr,
		void * buffer) {
	mat->nbc = nbc;
	mat->nbr = nbr;
	mat->line_step = nbc;
	if (buffer == NULL) {
		buffer = malloc(nbc * nbr * sizeof(double));
		memset(buffer, 0, nbc * nbr * sizeof(double));
	}
	if (buffer == NULL)
		return -1;
	mat->alloc_size = nbc * nbr;
	mat->data = (double *) buffer;
	mat->origin_r = 0;
	mat->origin_c = 0;

	return mat->alloc_size;
}

void free_double_matrix(struct double_matrix * mat) {
	if (mat->data != NULL) {
		free(mat->data);
	}
	mat->data = NULL;
}

void zeros_double_matrix(struct double_matrix * mat) {
	memset(mat->data, 0, mat->alloc_size * sizeof(double));
}

void get_double_sub_matrix(struct double_matrix * op, struct double_matrix * sub,
		unsigned int origin_nbc, unsigned int origin_nbr, unsigned int nbc,
		unsigned int nbr) {
	if (sub == NULL) {
		op->origin_r = origin_nbr;
		op->origin_c = origin_nbc;
		op->nbc = nbc;
		op->nbr = nbr;
	} else {
		int i;
		sub->origin_r = 0;
		sub->origin_c = 0;
		sub->nbc = nbc;
		sub->nbr = nbr;
		sub->line_step = sub->nbc;
		if (sub->data == NULL) {
			sub->data = malloc(nbc * nbr * sizeof(double));
			sub->alloc_size = nbc * nbr;
		}
		for (i = 0; i < nbr; i++) {
			memcpy(MAT_GET_ROW((*sub), i),
					&(MAT_ELT_AT((*op), origin_nbr+i, origin_nbc)),
					nbc * sizeof(double));
		}
	}
}

void identity_double_matrix(struct double_matrix * mat) {
	int i;
	for (i = 0; i < mat->nbc; i++)
		MAT_ELT_AT((*mat), i, i) = 1.0;
}

void random_double_matrix(struct double_matrix * mat) {
	int i, j;
	for (i = 0; i < mat->nbr; i++) {
		for (j = 0; j < mat->nbc; j++)
			MAT_ELT_AT((*mat), i, j) = (double) rand() / ((double) RAND_MAX);
	}
}

void print_double_matrix(struct double_matrix mat, char * name) {
	int i, j;
	printf("%s = [\n", name);
	for (i = 0; i < mat.nbr; i++) {
		for (j = 0; j < mat.nbc; j++) {
			printf("%lf\t", MAT_ELT_AT(mat, i, j));
		}
		printf(";\n");
	}
	printf("]\n");
}

void print_double_vec(double * vec, unsigned int size, char * name) {
	int j;
	printf("%s = [", name);
	for (j = 0; j < size; j++) {
		printf("%lf\t", vec[j]);
	}
	printf("]\n");
}

void copy_double_matrix(struct double_matrix * src, struct double_matrix * dest,
		char alloc) {
	int i;
	if (alloc) {
		alloc_double_matrix(dest, src->nbc, src->nbr, NULL);
	} else {
		alloc_double_matrix(dest, src->nbc, src->nbr, dest->data);
	}
	for (i = 0; i < src->nbr; i++) {
		memcpy(MAT_GET_ROW((*dest), i), MAT_GET_ROW((*src), i),
				src->nbc * sizeof(double));
	}
}
