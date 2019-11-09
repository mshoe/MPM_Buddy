#pragma once
#include "Constants.h"

bool InBounds(int node_i, int node_j, int x_bound, int y_bound);

real CubicBSpline(real x);
real CubicBSplineSlope(real x);

void PolarDecomp(const mat2& F, mat2& R, mat2& S);

void SVD(const mat2& R, const mat2& S, mat2& U, real& sig1, real& sig2, mat2& V);

double ExtractRotationAngle(mat2 R);