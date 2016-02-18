#include "lapack.h"

#include <cmath>

void Fgemm_(const char* trans_a, const char* trans_b, const int* m, const int* n, const int* k, const float* alpha, const float* a, const int* lda, const float* b, const int* ldb, const float* beta,
            float* c, const int* ldc) {
    int i1, i2, i3, beta_is_null, c_pos;
    int* a_1 = *trans_a == 't' ? &i3 : &i1;
    int* a_2 = *trans_a == 't' ? &i1 : &i3;
    int* b_1 = *trans_b == 't' ? &i2 : &i3;
    int* b_2 = *trans_b == 't' ? &i3 : &i2;
    beta_is_null = *beta == 0.0;
    for (i1 = 0; i1 < *m; ++i1)
        for (i2 = 0; i2 < *n; ++i2) {
            c_pos = i1 + i2 * *ldc;
            if (beta_is_null)
                c[c_pos] = 0.0;
            else
                c[c_pos] *= *beta;
            for (i3 = 0; i3 < *k; ++i3)
                c[c_pos] += *alpha * a[*a_1 + *a_2 * *lda] * b[*b_1 + *b_2 * *ldb];
        }
}

void Fgemv_(const char* trans, const int* m, const int* n, const float* alpha, const float* a, const int* lda, const float* x, const int* incx, const float* beta, float* y, const int* incy) {
    int i1, i3, beta_is_null, y_pos;
    int* a_1 = *trans == 't' ? &i3 : &i1;
    int* a_2 = *trans == 't' ? &i1 : &i3;
    beta_is_null = *beta == 0.0;
    for (i1 = 0; i1 < *m; ++i1) {
        y_pos = i1 * *incy;
        if (beta_is_null)
            y[y_pos] = 0.0;
        else
            y[y_pos] *= *beta;
        for (i3 = 0; i3 < *n; ++i3)
            y[y_pos] += *alpha * a[*a_1 + *a_2 * *lda] * x[i3 * *incx];
    }
}

void Flacpy_(const char* uplo, const int* m, const int* n, const float* a, const int* lda, float* b, const int* ldb) {
    int i1, i2;
    if (*uplo == 'u') {
        for (i1 = 0; i1 < *m; ++i1)
            for (i2 = i1; i2 < *n; ++i2)
                b[i1 + i2 * *ldb] = a[i1 + i2 * *lda];
    } else if (*uplo == 'l') {
        for (i1 = 0; i1 < *m; ++i1)
            for (i2 = (i1 < *n) ? i1 : *n - 1; i2 > -1; --i2)
                b[i1 + i2 * *ldb] = a[i1 + i2 * *lda];
    } else {
        for (i1 = 0; i1 < *m; ++i1)
            for (i2 = 0; i2 < *n; ++i2)
                b[i1 + i2 * *ldb] = a[i1 + i2 * *lda];
    }
}

int dpotrf_help_(int n, float* a, int ld1, int ld2) {
    int i1, i2, i3, curr_pos;
    float helper;
    for (i1 = 0; i1 < n; ++i1) {
        /* nondiagonal elements */
        for (i2 = 0; i2 < i1; ++i2) {
            curr_pos = i1 * ld1 + i2 * ld2;
            for (i3 = 0; i3 < i2; ++i3)
                a[curr_pos] -= a[i1 * ld1 + i3 * ld2] * a[i2 * ld1 + i3 * ld2];
            a[curr_pos] /= a[i2 * (ld1 + ld2)];
        }

        curr_pos = i1 * (ld1 + ld2);

        /* diagonal element */
        for (i2 = 0; i2 < i1; ++i2) {
            helper = a[i1 * ld1 + i2 * ld2];
            a[curr_pos] -= helper * helper;
        }
        if (!(a[curr_pos] > 0.0))
            return 1;
        a[curr_pos] = sqrt(a[curr_pos]);
    }
    return 0;
}

void Fpotrf_(const char* uplo, const int* n, float* a, const int* lda, int* info) {
    if (*uplo == 'u')
        *info = dpotrf_help_(*n, a, *lda, 1);
    else
        *info = dpotrf_help_(*n, a, 1, *lda);
}

void Fgetrf_swap_rows_(int cols, float* a, int lda, int r1, int r2) {
    float buffer;
    cols *= lda;
    for (; r1 < cols; r1 += lda, r2 += lda) {
        buffer = a[r1];
        a[r1] = a[r2];
        a[r2] = buffer;
    }
}

float abs_val(float x) {
    return (x < 0.0) ? -x : x;
}

void Fgetrf_pivotize_(int m, int n, float* a, int lda, int* ipiv) {
    int i, j, col_offs, max_row, buffer;

    for (j = 0; j < n; ++j) {
        ipiv[j] = j;
    }

    for (j = 0; j < n; ++j) {
        col_offs = j * lda;
        max_row = j;

        /* Find max within a column */
        for (i = j + 1; i < m; ++i)
            if (abs_val(a[col_offs + ipiv[i]]) > abs_val(a[col_offs + ipiv[max_row]]))
                max_row = i;

        if (max_row != j) {
            Fgetrf_swap_rows_(n, a, lda, ipiv[max_row], ipiv[j]);
            buffer = ipiv[max_row];
            ipiv[max_row] = ipiv[j];
            ipiv[j] = buffer;
        }
    }
}

void Fgetrf_(const int* m, const int* n, float* a, const int* lda, int* ipiv, int* info) {
    int i, j, k;

    *info = 0;
    /* only accepts square matrices, since we're using it only for inversion */
    if (*m != *n) {
        *info = 1;
        return;
    }

    Fgetrf_pivotize_(*m, *n, a, *lda, ipiv);

    for (j = 0; j < *n; ++j) {
        /* U */
        for (i = 0; i <= j; ++i) {
            for (k = 0; k < i; ++k)
                a[i + j * *lda] -= a[k + j * *lda] * a[i + k * *lda];
        }
        if (a[j + j * *lda] == 0.0 || a[j + j * *lda] == -0.0) {
            *info = 1;
            return;
        }

        /* L */
        for (i = j + 1; i < *m; ++i) {
            for (k = 0; k < j; ++k)
                a[i + j * *lda] -= a[k + j * *lda] * a[i + k * *lda];
            a[i + j * *lda] /= a[j + j * *lda];
        }
    }
}

void Fgetri_(const int* n, float* a, const int* lda, const int* ipiv, float* workspace, const int* len_workspace, int* info) {
    int i, j, col, col_offs_ws, col_offs_a;

    *info = 0;
    if (*len_workspace < *n * *n) {
        *info = 1;
        return;
    }

    /* set workspace to identity */
    for (i = 0; i < *len_workspace; ++i)
        workspace[i] = 0.0;
    for (i = 0; i < *n; ++i)
        workspace[i * (*n + 1)] = 1;

    for (i = 0; i < *n; ++i)
        if (a[i * (*lda + 1)] == 0.0 || a[i * (*lda + 1)] == -0.0) {
            *info = 1;
            return;
        }

    /* solve L * X = I */
    for (col = 0; col < *n; ++col) {
        col_offs_ws = col * *n;
        for (i = 0; i < *n; ++i)
            for (j = 0; j < i; ++j)
                workspace[col_offs_ws + i] -= a[i + j * *lda] * workspace[col_offs_ws + j];
    }

    /* solve U * A^-1 = X */
    for (col = 0; col < *n; ++col) {
        col_offs_ws = col * *n;
        for (i = *n - 1; i > -1; --i) {
            for (j = i + 1; j < *n; ++j)
                workspace[col_offs_ws + i] -= a[i + j * *lda] * workspace[col_offs_ws + j];
            workspace[col_offs_ws + i] /= a[i * (*lda + 1)];
        }
    }

    /* depivotize */
    for (j = 0; j < *n; ++j) {
        col_offs_a = ipiv[j] * *lda;
        col_offs_ws = j * *n;
        for (i = 0; i < *n; ++i)
            a[col_offs_a + i] = workspace[col_offs_ws + i];
    }
}

void Fger_(const int* m, const int* n, const float* alpha, const float* x, const int* incx, const float* y, const int* incy, float* a, const int* lda) {
    int i1, i2;
    for (i1 = 0; i1 < *m; ++i1)
        for (i2 = 0; i2 < *n; ++i2)
            a[i1 + i2 * *lda] += *alpha * x[i1 * *incx] * y[i2 * *incy];
}
