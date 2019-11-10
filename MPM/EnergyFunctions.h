#pragma once
#include "Constants.h"
#include "MpmFunctions.h"

namespace mpm {
	mat2 NeoHookeanPKTensor(mat2 Fe, real lam, real mew) {
		mat2 Fit = glm::transpose(glm::inverse(Fe));
		real J = glm::determinant(Fe);
		real logJ = real(glm::log(J));
		mat2 P = mew * (Fe - Fit) + lam * logJ * Fit;
		return P;
	}

	mat2 FixedCorotationalElasticityPKTensor(mat2 Fe, real lam, real mew)
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
