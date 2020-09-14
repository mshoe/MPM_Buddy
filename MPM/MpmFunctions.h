#pragma once
#include "Constants.h"

namespace mpm {
	const real Dp_inv_cubic = 3.0;
	const real Dp_inv_quadratic = 4.0;
	



	bool InBounds(size_t node_i, size_t node_j, size_t x_bound, size_t y_bound);


	double LinearShape(double x);
	double LinearShapeSlope(double x);

	double QuadraticBSpline(double x);
	double QuadraticBSplineSlope(double x);

	double CubicBSpline(double x);
	double CubicBSplineSlope(double x);

	void PolarDecomp(const mat2& F, mat2& R, mat2& S);

	void SVD(const mat2& R, const mat2& S, mat2& U, real& sig1, real& sig2, mat2& V);

	double ExtractRotationAngle(mat2 R);

	real InnerProduct(mat2 A, mat2 B);

	mat2 CofactorMatrix(mat2 X);

	mat2 NormalizedMatrix(mat2 X);

	real MatrixNorm(mat2 X);

	real MatrixNormSqrd(mat2 X);

	double Trace(mat2 X);
}