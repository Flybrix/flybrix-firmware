#ifndef SE_LAPACK_H_
#define SE_LAPACK_H_

/*
 * Matrix * Matrix multiplication
 * C = alpha * A * B + beta * C
 * A (m x k)
 * B (k x n)
 * C (m x n)
 */
void Fgemm_(const char* trans_a, const char* trans_b, const int* m, const int* n, const int* k, const float* alpha, const float* a, const int* lda, const float* b, const int* ldb, const float* beta,
            float* c, const int* ldc);

/*
 * Matrix * Vector multiplication
 * y = alpha * A * x + beta * y
 * A (m x n)
 * x (n)
 * y (m)
 */
void Fgemv_(const char* trans, const int* m, const int* n, const float* alpha, const float* a, const int* lda, const float* x, const int* incx, const float* beta, float* y, const int* incy);

/*
 * Copy Matrix
 */
void Flacpy_(const char* uplo, const int* m, const int* n, const float* a, const int* lda, float* b, const int* ldb);

/*
 * Cholesky factorization of a symmetric positive definite NxN matrix A
 * The matrix A is replaced with matrix X, that satisfies, for the original A:
 * A = X * X' for "l"
 * A = X' * X for "u"
 * Nonzero info means an error occured
 * WARNING: The leftover part of the matrix, that is expected to be all zeros
 *          in the resulting matrix factorization, is not replaced!
 */
void Fpotrf_(const char* uplo, const int* n, float* a, const int* lda, int* info);

/* LU decomoposition of a general matrix */
void Fgetrf_(const int* m, const int* n, float* a, const int* lda, int* ipiv, int* info);

/* generate inverse of a matrix given its LU decomposition */
void Fgetri_(const int* n, float* a, const int* lda, const int* ipiv, float* workspace, const int* len_workspace, int* info);

/* Perform the rank 1 operation: A += alpha * x * y' */
void Fger_(const int* m, const int* n, const float* alpha, const float* x, const int* incx, const float* y, const int* incy, float* a, const int* lda);

#endif /* end of include guard: SE_LAPACK_H_ */
