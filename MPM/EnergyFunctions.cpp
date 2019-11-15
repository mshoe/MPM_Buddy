#include "EnergyFunctions.h"
#include <iostream>

namespace mpm {
	mat2 NeoHookeanPKTensor(mat2 Fe, real lam, real mew) {
		mat2 Fit = glm::transpose(glm::inverse(Fe));
		real J = glm::determinant(Fe);
		real logJ = real(glm::log(J));
		mat2 P = mew * (Fe - Fit) + lam * logJ * Fit;
		return P;
	}

	mat2 SimpleSnowPKTensor(mat2& Fe, mat2& Fp, real mew, real lam, real crit_c, real crit_s, real hardening) {
		mat2 F_total = Fe * Fp;

		// calculate the polar decomposition Fe = RU.
		// then use polar decomp to calculate SVD of Fe

		mat2 R, S;
		PolarDecomp(Fe, R, S);

		mat2 U, V;
		real sig1, sig2;
		SVD(R, S, U, sig1, sig2, V);

		sig1 = glm::clamp(sig1, 1.0 - crit_c, 1.0 + crit_s);
		sig2 = glm::clamp(sig2, 1.0 - crit_c, 1.0 + crit_s);

		mat2 sigMat = mat2(sig1, 0.0, 0.0, sig2);

		// finally calculate new Fe and Fp
		Fe = U * sigMat * transpose(V);
		Fp = V * inverse(sigMat) * transpose(U) * F_total;

		mat2 Feit = glm::transpose(glm::inverse(Fe));
		real Je = glm::determinant(Fe);
		real Jp = glm::determinant(Fp);

		real pcof = real(glm::exp(hardening * (1.0 - Jp)));

		mat2 P = 2.0 * mew * (Fe - R) + lam * (Je - 1.0) * Je * Feit;
		P *= pcof;
		return P;
	}
}

mat2 mpm::FixedCorotationalElasticity::PKTensor(mat2 Fe, real lam, real mew)
{
	mat2 Fit = glm::transpose(glm::inverse(Fe));
	real J = glm::determinant(Fe);

	// calculate the polar decomposition F = RU.
	// Then calculate P using F and R
	mat2 R, S;
	PolarDecomp(Fe, R, S);

	mat2 P = 2.0 * mew * (Fe - R) + lam * (J - 1.0) * J * Fit;
	return P;
}

mat4 mpm::FixedCorotationalElasticity::d2Psi_dF2_Mat4(mat2 Fe, real lam, real mew)
{
	mat2 R, S, U, V;
	real s1, s2;
	PolarDecomp(Fe, R, S);
	SVD(R, S, U, s1, s2, V);

	real u1 = ExtractRotationAngle(U);
	real v1 = ExtractRotationAngle(V);

	// pooped out from MATLAB script: MATLAB/P_SVD.mlx
	// Assuming release mode will optimize this bad boy

	real cosu1 = cos(u1);
	real sinu1 = sin(u1);
	real cosv1 = cos(v1);
	real sinv1 = sin(v1);

	mat4 dPdF;
	dPdF[0][0] = mew * cosu1 * cosv1 * 2.0 - lam * sinu1 * sinv1 + lam * (s2 * s2) * cosu1 * cosv1 + lam * s1 * s2 * sinu1 * sinv1 * 2.0;
	dPdF[0][1] = lam * cosv1 * sinu1 + mew * cosu1 * sinv1 * 2.0 + lam * (s2 * s2) * cosu1 * sinv1 - lam * s1 * s2 * cosv1 * sinu1 * 2.0;
	dPdF[0][2] = lam * cosu1 * sinv1 + mew * cosv1 * sinu1 * 2.0 + lam * (s2 * s2) * cosv1 * sinu1 - lam * s1 * s2 * cosu1 * sinv1 * 2.0;
	dPdF[0][3] = -lam * cosu1 * cosv1 + mew * sinu1 * sinv1 * 2.0 + lam * (s2 * s2) * sinu1 * sinv1 + lam * s1 * s2 * cosu1 * cosv1 * 2.0;
	dPdF[1][0] = -(mew * cosu1 * sinv1 * -2.0 + mew * cosv1 * sinu1 * 2.0 + lam * s1 * cosv1 * sinu1 + lam * s2 * cosv1 * sinu1 + mew * s1 * cosu1 * sinv1 * 2.0 + mew * s2 * cosu1 * sinv1 * 2.0 - lam * s1 * (s2 * s2) * cosv1 * sinu1 - lam * (s1 * s1) * s2 * cosv1 * sinu1) / (s1 + s2);
	dPdF[1][1] = -(mew * cosu1 * cosv1 * 2.0 + mew * sinu1 * sinv1 * 2.0 + lam * s1 * sinu1 * sinv1 + lam * s2 * sinu1 * sinv1 - mew * s1 * cosu1 * cosv1 * 2.0 - mew * s2 * cosu1 * cosv1 * 2.0 - lam * s1 * (s2 * s2) * sinu1 * sinv1 - lam * (s1 * s1) * s2 * sinu1 * sinv1) / (s1 + s2);
	dPdF[1][2] = (mew * cosu1 * cosv1 * 2.0 + mew * sinu1 * sinv1 * 2.0 - mew * s1 * sinu1 * sinv1 * 2.0 - mew * s2 * sinu1 * sinv1 * 2.0 + lam * s1 * cosu1 * cosv1 + lam * s2 * cosu1 * cosv1 - lam * s1 * (s2 * s2) * cosu1 * cosv1 - lam * (s1 * s1) * s2 * cosu1 * cosv1) / (s1 + s2);
	dPdF[1][3] = (mew * cosu1 * sinv1 * 2.0 - mew * cosv1 * sinu1 * 2.0 + lam * s1 * cosu1 * sinv1 + lam * s2 * cosu1 * sinv1 + mew * s1 * cosv1 * sinu1 * 2.0 + mew * s2 * cosv1 * sinu1 * 2.0 - lam * s1 * (s2 * s2) * cosu1 * sinv1 - lam * (s1 * s1) * s2 * cosu1 * sinv1) / (s1 + s2);
	dPdF[2][0] = -(mew * cosu1 * sinv1 * 2.0 - mew * cosv1 * sinu1 * 2.0 + lam * s1 * cosu1 * sinv1 + lam * s2 * cosu1 * sinv1 + mew * s1 * cosv1 * sinu1 * 2.0 + mew * s2 * cosv1 * sinu1 * 2.0 - lam * s1 * (s2 * s2) * cosu1 * sinv1 - lam * (s1 * s1) * s2 * cosu1 * sinv1) / (s1 + s2);
	dPdF[2][1] = (mew * cosu1 * cosv1 * 2.0 + mew * sinu1 * sinv1 * 2.0 - mew * s1 * sinu1 * sinv1 * 2.0 - mew * s2 * sinu1 * sinv1 * 2.0 + lam * s1 * cosu1 * cosv1 + lam * s2 * cosu1 * cosv1 - lam * s1 * (s2 * s2) * cosu1 * cosv1 - lam * (s1 * s1) * s2 * cosu1 * cosv1) / (s1 + s2);
	dPdF[2][2] = -(mew * cosu1 * cosv1 * 2.0 + mew * sinu1 * sinv1 * 2.0 + lam * s1 * sinu1 * sinv1 + lam * s2 * sinu1 * sinv1 - mew * s1 * cosu1 * cosv1 * 2.0 - mew * s2 * cosu1 * cosv1 * 2.0 - lam * s1 * (s2 * s2) * sinu1 * sinv1 - lam * (s1 * s1) * s2 * sinu1 * sinv1) / (s1 + s2);
	dPdF[2][3] = (mew * cosu1 * sinv1 * -2.0 + mew * cosv1 * sinu1 * 2.0 + lam * s1 * cosv1 * sinu1 + lam * s2 * cosv1 * sinu1 + mew * s1 * cosu1 * sinv1 * 2.0 + mew * s2 * cosu1 * sinv1 * 2.0 - lam * s1 * (s2 * s2) * cosv1 * sinu1 - lam * (s1 * s1) * s2 * cosv1 * sinu1) / (s1 + s2);
	dPdF[3][0] = -lam * cosu1 * cosv1 + mew * sinu1 * sinv1 * 2.0 + lam * (s1 * s1) * sinu1 * sinv1 + lam * s1 * s2 * cosu1 * cosv1 * 2.0;
	dPdF[3][1] = -lam * cosu1 * sinv1 - mew * cosv1 * sinu1 * 2.0 - lam * (s1 * s1) * cosv1 * sinu1 + lam * s1 * s2 * cosu1 * sinv1 * 2.0;
	dPdF[3][2] = -lam * cosv1 * sinu1 - mew * cosu1 * sinv1 * 2.0 - lam * (s1 * s1) * cosu1 * sinv1 + lam * s1 * s2 * cosv1 * sinu1 * 2.0;
	dPdF[3][3] = mew * cosu1 * cosv1 * 2.0 - lam * sinu1 * sinv1 + lam * (s1 * s1) * cosu1 * cosv1 + lam * s1 * s2 * sinu1 * sinv1 * 2.0;

	return dPdF; // NOTE: glm is column-major. Accessing from this matrix will be the same, but multiplication will need to be done using the transpose of this
}

mat2 mpm::FixedCorotationalElasticity::d2Psi_dF2_multbydF(mat2 Fe, real mew, real lam, mat2 dF)
{
	mat2 R, S;
	PolarDecomp(Fe, R, S);
	real J = glm::determinant(Fe);
	mat2 Fit = glm::transpose(glm::inverse(Fe));

	mat2 A = mat2(0.0);

	A += lam * J * Fit * InnerProduct(J * Fit, dF);

	A += lam * (J - 1.0) * CofactorMatrix(dF);

	A += 2.0 * mew * dF;

	real b = R[1][0] * dF[0][0] + R[1][1] * dF[0][1] - (dF[1][0] * R[0][0] + dF[1][1] * R[0][1]);
	real a = b / (S[0][0] + S[1][1]);
	mat2 dR = R * mat2(0, a, -a, 0); // column major

	A -= 2.0 * mew * dR;

	return A;
}

mat4 mpm::FixedCorotationalElasticity::d2Psi_dF2_Mat4_trick(mat2 Fe, real mew, real lam)
{
	mat4 dPdF;

	// using column-major notation to make life easier

	static const mat2 m00(1.0, 0.0, 0.0, 0.0);
	static const mat2 m01(0.0, 1.0, 0.0, 0.0);

	//std::cout << glm::to_string(m01) << std::endl;
	//std::cout << m01[0][1] << std::endl;
	static const mat2 m10(0.0, 0.0, 1.0, 0.0);
	static const mat2 m11(0.0, 0.0, 0.0, 1.0);

	// |1, 0|
	// |0, 0|
	mat2 dPdF00 = d2Psi_dF2_multbydF(Fe, mew, lam, m00);

	// NOTE: My math says these are the correct matrices, but when I run the code, the other way is correct
	//// |0, 0|
	//// |1, 0|
	//mat2 dPdF01 = d2Psi_dF2_multbydF(Fe, mew, lam, m01);

	//// |0, 1|
	//// |0, 0|
	//mat2 dPdF10 = d2Psi_dF2_multbydF(Fe, mew, lam, m10);


	// swap these around??? did I make a mistake somewhere
	// |0, 0|
	// |1, 0|
	mat2 dPdF10 = d2Psi_dF2_multbydF(Fe, mew, lam, m01);

	// |0, 1|
	// |0, 0|
	mat2 dPdF01 = d2Psi_dF2_multbydF(Fe, mew, lam, m10);

	// |0, 0|
	// |0, 1|
	mat2 dPdF11 = d2Psi_dF2_multbydF(Fe, mew, lam, m11);

	/*dPdF[0][0] = dPdF00[0][0];
	dPdF[0][1] = dPdF00[0][1];
	dPdF[1][0] = dPdF00[1][0];
	dPdF[1][1] = dPdF00[1][1];

	dPdF[2 + 0][0] = dPdF10[0][0];
	dPdF[2 + 0][1] = dPdF10[0][1];
	dPdF[2 + 1][0] = dPdF10[1][0];
	dPdF[2 + 1][1] = dPdF10[1][1];

	dPdF[0][2 + 0] = dPdF01[0][0];
	dPdF[0][2 + 1] = dPdF01[0][1];
	dPdF[1][2 + 0] = dPdF01[1][0];
	dPdF[1][2 + 1] = dPdF01[1][1];

	dPdF[2 + 0][2 + 0] = dPdF11[0][0];
	dPdF[2 + 0][2 + 1] = dPdF11[0][1];
	dPdF[2 + 1][2 + 0] = dPdF11[1][0];
	dPdF[2 + 1][2 + 1] = dPdF11[1][1];*/

	// assume column-major flattening (shouldn't matter since this should be symmetric)
	dPdF[0][0] = dPdF00[0][0];
	dPdF[0][1] = dPdF00[1][0];
	dPdF[0][2] = dPdF00[0][1];
	dPdF[0][3] = dPdF00[1][1];

	dPdF[1][0] = dPdF01[0][0];
	dPdF[1][1] = dPdF01[1][0];
	dPdF[1][2] = dPdF01[0][1];
	dPdF[1][3] = dPdF01[1][1];

	dPdF[2][0] = dPdF10[0][0];
	dPdF[2][1] = dPdF10[1][0];
	dPdF[2][2] = dPdF10[0][1];
	dPdF[2][3] = dPdF10[1][1];

	dPdF[3][0] = dPdF11[0][0];
	dPdF[3][1] = dPdF11[1][0];
	dPdF[3][2] = dPdF11[0][1];
	dPdF[3][3] = dPdF11[1][1];

	return dPdF;
}
