#pragma once
#include <array>
#include <initializer_list>
#include <type_traits>

template <typename T, int M, int N>
class Matrix {
	std::array<T, M*N> a;

	// Uninitialized constructor
	Matrix(nullptr_t _ [[maybe_unused]]) {}

public:
	Matrix() { a.fill(0); };
	Matrix(const Matrix<T, M, N> &other) = default;
	Matrix(std::initializer_list<T> l) {
		size_t i = 0;
		for (const T x : l) {
			a[i++] = x;
		}
	}

	static Matrix<T, M, N> uninitialized() {
		return Matrix<T, M, N>(nullptr);
	}

	T &operator () (size_t i, size_t j) { return a[i * N + j]; }
	const T &operator () (size_t i, size_t j) const { return a[i * N + j]; }

	// Only enable single-argument getters for vectors
	template <int X=M, int Y=N>
	typename std::enable_if<X == 1 || Y == 1, T&>::type operator () (size_t i) { return a[i]; }
	template <int X=M, int Y=N>
	typename std::enable_if<X == 1 || Y == 1, const T&>::type operator () (size_t i) const { return a[i]; }

	Matrix<T, N, M> transposed() const {
		Matrix<T, N, M> t = Matrix<T, N, M>::uninitialized();
		for (size_t i = 0; i < M; ++i){
			for (size_t j = 0; j < N; ++j){
				t(j, i) = (*this)(i, j);
			}
		}
		return t;
	}

	template <int P>
	Matrix<T, M, P> operator * (const Matrix<T, N, P> &other) const {
		Matrix<T, M, P> m;
		for (size_t i = 0; i < M; ++i) {
			for (size_t j = 0; j < P; ++j) {
				for (size_t k = 0; k < N; ++k) {
					m(i, j) += (*this)(i, k) * other(k, j);
				}
			}
		}
		return m;
	}

	Matrix<T, M, N> operator + (const Matrix<T, M, N> &other) const {
		Matrix<T, M, N> m = Matrix<T, M, N>::uninitialized();
		for (size_t i = 0; i < M; ++i) {
			for (size_t j = 0; j < N; ++j) {
				m(i, j) = (*this)(i, j) + other(i, j);
			}
		}
		return m;
	}

	Matrix<T, M, N> operator - (const Matrix<T, M, N> &other) const {
		Matrix<T, M, N> m = Matrix<T, M, N>::uninitialized();
		for (size_t i = 0; i < M; ++i) {
			for (size_t j = 0; j < N; ++j) {
				m(i, j) = (*this)(i, j) - other(i, j);
			}
		}
		return m;
	}

	T norm_sq() const {
		T sq_sum = 0;
		for (const T x : a) {
			sq_sum += x * x;
		}
		return sq_sum;
	}
};

template <typename T, int M>
using Vector = Matrix<T, M, 1>;
