#include "MpmFunctions.h"

bool mpm::InBounds(int node_i, int node_j, int x_bound, int y_bound) {
	return (node_i >= 0 && node_i < x_bound && node_j >= 0 && node_j < y_bound);
}

real mpm::CubicBSpline(real x) {
	using glm::step;
	x = abs(x);
	if (0.0 <= x && x < 1.0) {
		return 0.5 * x * x * x - x * x + 2.0 / 3.0;
	}
	else if (1.0 <= x && x < 2.0) {
		return (2.0 - x) * (2.0 - x) * (2.0 - x) / 6.0;
	}
	else {
		return 0.0;
	}
}

real mpm::CubicBSplineSlope(real x) {
	using glm::step;
	// step(x) is 0 <= x means 1.0

	real absx = abs(x);
	return (absx < 1.0) ? step(0.0, absx) * (1.5 * x * absx - 2 * x) :
		step(absx, 2.0) * (-x * absx / 2 + 2 * x - 2 * x / absx);
}

void mpm::PolarDecomp(const mat2& F, mat2& R, mat2& S) {
	// calculate the polar decomposition F = RS.

	real x = F[0][0] + F[1][1];
	real y = F[0][1] - F[1][0]; // glsl is column major. This is really F_21 - F_12 in row major notation

	real d = sqrt(x * x + y * y);

	real c = x / d;
	real s = -y / d;
	R = (d == 0.0) ? mat2(1.0, 0.0, 0.0, 1.0) : mat2(c, -s, s, c);
	S = transpose(R) * F;
}

void mpm::SVD(const mat2& R, const mat2& S, mat2& U, real& sig1, real& sig2, mat2& V) {

	// check if S is diagonal (S is symmetric for sure)
	double c_hat, s_hat;

	if (S[1][0] == 0) {
		c_hat = 1.0;
		s_hat = 0.0;
		sig1 = S[0][0];
		sig2 = S[1][1];
	}
	else {
		double tau = 0.5 * (S[0][0] - S[1][1]);
		double w = sqrt(tau * tau + S[1][0] * S[1][0]);
		double t = (tau > 0) ? S[1][0] / (tau + w) : S[1][0] / (tau - w);
		c_hat = 1.0 / sqrt(t * t + 1.0);
		s_hat = -t * c_hat;
		sig1 = c_hat * c_hat * S[0][0] - 2.0 * c_hat * s_hat * S[1][0] + s_hat * s_hat * S[1][1];
		sig2 = s_hat * s_hat * S[0][0] + 2.0 * c_hat * s_hat * S[1][0] + c_hat * c_hat * S[1][1];
	}

	double c, s;
	if (sig1 < sig2) {
		// swap the singular values so sig1 > sig2
		double temp = sig1;
		sig1 = sig2;
		sig2 = temp;
		c = -s_hat;
		s = c_hat;
	}
	else {
		c = c_hat;
		s = s_hat;
	}
	V = mat2(c, -s, s, c);
	U = R * V;
}

double mpm::ExtractRotationAngle(mat2 R) {
	// assume R is a rotation matrix
	/*
	|cos(a), -sin(a)|
	|sin(a), cos(a) |
	*/

	return atan2(R[0][1], R[0][0]); // glm is column major
}

real mpm::InnerProduct(mat2 A, mat2 B)
{
	return A[0][0] * B[0][0] + A[1][0] * B[1][0] + A[0][1] * B[0][1] + A[1][1] * B[1][1];
}

mat2 mpm::CofactorMatrix(mat2 X)
{
	mat2 cofmat;
	cofmat[0][0] = X[1][1];
	cofmat[0][1] = -X[1][0];
	cofmat[1][0] = -X[0][1];
	cofmat[1][1] = X[0][0];
	return cofmat;
}

mat2 mpm::NormalizedMatrix(mat2 X)
{
	real X_len = X[0][0] * X[0][0] +
		X[1][0] * X[1][0] +
		X[0][1] * X[0][1] +
		X[1][1] * X[1][1];
	X_len = sqrt(X_len);

	if (X_len == 0.0)
		return mat2(0.0);
	else
		return X / X_len;
}

real mpm::MatrixNorm(mat2 X)
{
	return sqrt(X[0][0] * X[0][0] +
		X[1][0] * X[1][0] +
		X[0][1] * X[0][1] +
		X[1][1] * X[1][1]);
}
