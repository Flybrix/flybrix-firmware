#ifndef LINALG_IMPL_H
#define LINALG_IMPL_H

#include "linalg.h"

#include <cmath>

template <typename T, size_t FIELDS>
Vector<T, FIELDS>::Vector() : data{0} {
}

template <typename T, size_t FIELDS>
Vector<T, FIELDS>::Vector(const Vector<T, FIELDS>& v) {
    for (size_t i = 0; i < FIELDS; ++i) {
        data[i] = v.data[i];
    }
}

template <typename T, size_t FIELDS>
Vector<T, FIELDS>::Vector(Vector<T, FIELDS>&& v) {
    for (size_t i = 0; i < FIELDS; ++i) {
        data[i] = v.data[i];
    }
}

template <typename T, size_t FIELDS>
Vector<T, FIELDS>& Vector<T, FIELDS>::operator=(const Vector<T, FIELDS>& v) {
    for (size_t i = 0; i < FIELDS; ++i) {
        data[i] = v.data[i];
    }
    return *this;
}

template <typename T, size_t FIELDS>
Vector<T, FIELDS>& Vector<T, FIELDS>::operator=(Vector<T, FIELDS>&& v) {
    for (size_t i = 0; i < FIELDS; ++i) {
        data[i] = v.data[i];
    }
    return *this;
}

template <typename T, size_t FIELDS>
Vector<T, FIELDS>& Vector<T, FIELDS>::operator+=(const Vector<T, FIELDS>& v) {
    for (size_t i = 0; i < FIELDS; ++i) {
        data[i] += v.data[i];
    }
    return *this;
}

template <typename T, size_t FIELDS>
Vector<T, FIELDS>& Vector<T, FIELDS>::operator-=(const Vector<T, FIELDS>& v) {
    for (size_t i = 0; i < FIELDS; ++i) {
        data[i] -= v.data[i];
    }
    return *this;
}

template <typename T, size_t FIELDS>
Vector<T, FIELDS> operator+(const Vector<T, FIELDS>& u, const Vector<T, FIELDS>& v) {
    Vector<T, FIELDS> result{u};
    result += v;
    return result;
}

template <typename T, size_t FIELDS>
Vector<T, FIELDS> operator-(const Vector<T, FIELDS>& u, const Vector<T, FIELDS>& v) {
    Vector<T, FIELDS> result{u};
    result -= v;
    return result;
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>::Matrix() : data{0} {
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>::Matrix(const Matrix<T, ROWS, COLS>& v) {
    for (size_t i = 0; i < ROWS * COLS; ++i) {
        data[i] = v.data[i];
    }
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>::Matrix(Matrix<T, ROWS, COLS>&& v) {
    for (size_t i = 0; i < ROWS * COLS; ++i) {
        data[i] = v.data[i];
    }
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>& Matrix<T, ROWS, COLS>::operator=(const Matrix<T, ROWS, COLS>& v) {
    for (size_t i = 0; i < ROWS * COLS; ++i) {
        data[i] = v.data[i];
    }
    return *this;
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>& Matrix<T, ROWS, COLS>::operator=(Matrix<T, ROWS, COLS>&& v) {
    for (size_t i = 0; i < ROWS * COLS; ++i) {
        data[i] = v.data[i];
    }
    return *this;
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>& Matrix<T, ROWS, COLS>::operator+=(const Matrix<T, ROWS, COLS>& v) {
    for (size_t i = 0; i < ROWS * COLS; ++i) {
        data[i] += v.data[i];
    }
    return *this;
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>& Matrix<T, ROWS, COLS>::operator-=(const Matrix<T, ROWS, COLS>& v) {
    for (size_t i = 0; i < ROWS * COLS; ++i) {
        data[i] -= v.data[i];
    }
    return *this;
}

template <typename T, size_t ROWS, size_t COLS>
Matrix<T, ROWS, COLS>& Matrix<T, ROWS, COLS>::operator*=(const Matrix<T, COLS, COLS>& v) {
    for (size_t i = 0; i < ROWS; ++i) {
        T row[COLS];
        for (size_t j = 0; j < COLS; ++j) {
            T sum{0};
            for (size_t k = 0; k < COLS; ++k) {
                sum += (*this)(i, k) * v(k, j);
            }
            row[j] = sum;
        }
        for (size_t j = 0; j < COLS; ++j) {
            (*this)(i, j) = row[j];
        }
    }
    return *this;
}

template <typename T, size_t ROWS, size_t COLS, size_t COLS_OTH>
Matrix<T, ROWS, COLS_OTH> operator*(const Matrix<T, ROWS, COLS>& u, const Matrix<T, COLS, COLS_OTH>& v) {
    Matrix<T, ROWS, COLS_OTH> result;
    for (size_t i = 0; i < ROWS; ++i) {
        for (size_t j = 0; j < COLS_OTH; ++j) {
            T sum{0};
            for (size_t k = 0; k < COLS; ++k) {
                sum += u(i, k) * v(k, j);
            }
            result(i, j) = sum;
        }
    }
    return result;
}

template <typename T, size_t ROWS, size_t COLS>
Vector<T, ROWS> operator*(const Matrix<T, ROWS, COLS>& u, const Vector<T, COLS>& v) {
    Vector<T, ROWS> result;
    for (size_t i = 0; i < ROWS; ++i) {
        for (size_t j = 0; j < COLS; ++j) {
            result[i] = u(i, j) * v[j];
        }
    }
    return result;
}

// https://en.wikipedia.org/wiki/Cholesky_decomposition#The_Cholesky.E2.80.93Banachiewicz_and_Cholesky.E2.80.93Crout_algorithms
template <typename T, size_t ROWS>
Matrix<T, ROWS, ROWS> cholesky(const Matrix<T, ROWS, ROWS>& v, T scaling) {
    Matrix<T, ROWS, ROWS> l{};
    for (size_t i = 0; i < ROWS; ++i) {
        T diag{v(i, i) * scaling};
        for (size_t j = 0; j < i; ++j) {
            T val{v(i, j) * scaling};
            for (size_t k = 0; k < j; ++k) {
                val -= l(i, k) * l(j, k);
            }
            val /= l(j, j);
            l(i, j) = val;
            diag -= val * val;
        }
        l(i, i) = sqrt(diag);
    }
    return l;
}

template <typename T, size_t ROWS>
static inline void solveLowerTriangleEquationSystem(const Matrix<T, ROWS, ROWS>& l, T data[ROWS]) {
    for (size_t i = 0; i < ROWS; ++i) {
        for (size_t j = 0; j < i; ++j) {
            data[i] -= data[j] * l(i, j);
        }
        data[i] /= l(i, i);
    }
}

template <typename T, size_t ROWS>
Matrix<T, ROWS, ROWS> invertRootable(const Matrix<T, ROWS, ROWS>& v) {
    const Matrix<T, ROWS, ROWS> l{cholesky<T, ROWS>(v, 1)};
    Matrix<T, ROWS, ROWS> l_inv_t;

    for (size_t i = 0; i < ROWS; ++i) {
        l_inv_t(i, i) = 1;
    }

    for (size_t i = 0; i < ROWS; ++i) {
        solveLowerTriangleEquationSystem(l, l_inv_t.data + i * ROWS);
    }

    return multMatrixAndTransposeMatrix(l_inv_t, l_inv_t, v);
}

template <typename T, size_t ROWS, size_t COLS>
void Matrix<T, ROWS, COLS>::addCorrelation(const Vector<T, ROWS>& u, const Vector<T, COLS>& v, T scaling) {
    for (size_t i = 0; i < ROWS; ++i) {
        for (size_t j = 0; j < COLS; ++j) {
            (*this)(i, j) = u[i] * v[j] * scaling;
        }
    }
}

template <typename T, size_t ROWS, size_t COLS, size_t ROWS_OTH>
Matrix<T, ROWS, ROWS_OTH> multMatrixAndTransposeMatrix(const Matrix<T, ROWS, COLS>& u, const Matrix<T, ROWS_OTH, COLS>& v) {
    Matrix<T, ROWS, ROWS_OTH> result;
    for (size_t i = 0; i < ROWS; ++i) {
        for (size_t j = 0; j < ROWS_OTH; ++j) {
            T sum{0};
            for (size_t k = 0; k < COLS; ++k) {
                sum += u(i, k) * v(j, k);
            }
            result(i, j) = sum;
        }
    }
    return result;
}

#endif  // LINALG_IMPL_H
