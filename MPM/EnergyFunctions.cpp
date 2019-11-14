#include "EnergyFunctions.h"

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

mat4 mpm::FixedCorotationalElasticity::d2Psi_dF2_Mat4(double mew, double lam, mat2 Fe)
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
