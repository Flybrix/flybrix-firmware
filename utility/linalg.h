#ifndef LINALG_H
#define LINALG_H

template <typename T, size_t FIELDS>
struct Vector {
    Vector();
    Vector(const Vector<T, FIELDS>& v);
    Vector(Vector<T, FIELDS>&& v);
    Vector& operator=(const Vector<T, FIELDS>& v);
    Vector& operator=(Vector<T, FIELDS>&& v);

    Vector<T, FIELDS>& operator+=(const Vector<T, FIELDS>& v);
    Vector<T, FIELDS>& operator-=(const Vector<T, FIELDS>& v);

    T operator[](size_t field) const {
        return data[field];
    }

    T& operator[](size_t field) {
        return data[field];
    }

    T data[FIELDS];
};

template <typename T, size_t FIELDS>
Vector<T, FIELDS> operator+(const Vector<T, FIELDS>& u, const Vector<T, FIELDS>& v);
template <typename T, size_t FIELDS>
Vector<T, FIELDS> operator-(const Vector<T, FIELDS>& u, const Vector<T, FIELDS>& v);

template <typename T, size_t ROWS, size_t COLS>
struct Matrix {
    Matrix();
    Matrix(const Matrix<T, ROWS, COLS>& v);
    Matrix(Matrix<T, ROWS, COLS>&& v);
    Matrix& operator=(const Matrix<T, ROWS, COLS>& v);
    Matrix& operator=(Matrix<T, ROWS, COLS>&& v);

    Matrix<T, ROWS, COLS>& operator+=(const Matrix<T, ROWS, COLS>& v);
    Matrix<T, ROWS, COLS>& operator-=(const Matrix<T, ROWS, COLS>& v);
    Matrix<T, ROWS, COLS>& operator*=(const Matrix<T, COLS, COLS>& v);

    T operator()(size_t row, size_t col) const {
        return data[row * COLS + col];
    }

    T& operator()(size_t row, size_t col) {
        return data[row * COLS + col];
    }

    void addCorrelation(const Vector<T, ROWS>& u, const Vector<T, COLS>& v, T scaling);

    T data[ROWS * COLS];
};

template <typename T, size_t ROWS>
Matrix<T, ROWS, ROWS> cholesky(const Matrix<T, ROWS, ROWS>& v, T scaling);

template <typename T, size_t ROWS>
void invertRootable(Matrix<T, ROWS, ROWS>& v);

template <typename T, size_t ROWS, size_t COLS, size_t COLS_OTH>
Matrix<T, ROWS, COLS_OTH> operator*(const Matrix<T, ROWS, COLS>& u, const Matrix<T, COLS, COLS_OTH>& v);

template <typename T, size_t ROWS, size_t COLS, size_t ROWS_OTH>
void multMatrixAndTransposeMatrix(const Matrix<T, ROWS, COLS>& u, const Matrix<T, ROWS_OTH, COLS>& v, Matrix<T, ROWS, ROWS_OTH>& result);

template <typename T, size_t ROWS, size_t COLS>
Vector<T, ROWS> operator*(const Matrix<T, ROWS, COLS>& u, const Vector<T, COLS>& v);

#include "linalg_impl.h"

#endif  // LINALG_H
