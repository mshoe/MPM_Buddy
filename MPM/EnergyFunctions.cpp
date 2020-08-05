#include "EnergyFunctions.h"
#include <iostream>

namespace mpm {
	mat2 NeoHookeanPKTensor(mat2 Fe, double lam, double mew) {
		mat2 Fit = glm::transpose(glm::inverse(Fe));
		double J = glm::determinant(Fe);
		double logJ = double(glm::log(J));
		mat2 P = mew * (Fe - Fit) + lam * logJ * Fit;
		return P;
	}

	mat2 SimpleSnowPKTensor(mat2& Fe, mat2& Fp, double mew, double lam, double crit_c, double crit_s, double hardening) {
		mat2 F_total = Fe * Fp;

		// calculate the polar decomposition Fe = RU.
		// then use polar decomp to calculate SVD of Fe

		mat2 R, S;
		PolarDecomp(Fe, R, S);

		mat2 U, V;
		double sig1, sig2;
		SVD(R, S, U, sig1, sig2, V);

		sig1 = glm::clamp(sig1, 1.0 - crit_c, 1.0 + crit_s);
		sig2 = glm::clamp(sig2, 1.0 - crit_c, 1.0 + crit_s);

		mat2 sigMat = mat2(sig1, 0.0, 0.0, sig2);

		// finally calculate new Fe and Fp
		Fe = U * sigMat * transpose(V);
		Fp = V * inverse(sigMat) * transpose(U) * F_total;

		mat2 Feit = glm::transpose(glm::inverse(Fe));
		double Je = glm::determinant(Fe);
		double Jp = glm::determinant(Fp);

		double pcof = double(glm::exp(hardening * (1.0 - Jp)));

		mat2 P = 2.0 * mew * (Fe - R) + lam * (Je - 1.0) * Je * Feit;
		P *= pcof;
		return P;
	}

	mat3 ElasticityMatrix(double E, double nu, bool stress_state)
	{
		/*
		function C = elasticityMatrix(E0, nu0, stressState)
			%
			%Elasticity matrix for isotropic elastic materials.
			%
			% VP Nguyen
			% Cardiff University, UK

			if (strcmp(stressState, 'PLANE_STRESS')) % Plane Stress case
				C = E0 / (1 - nu0 ^ 2) * [1      nu0         0;
				nu0     1          0;
				0       0  (1 - nu0) / 2];
			elseif(strcmp(stressState, 'PLANE_STRAIN')) % Plane Strain case
				C = E0 / (1 + nu0) / (1 - 2 * nu0) * [1 - nu0   nu0        0;
														nu0    1 - nu0       0;
														0      0  1 / 2 - nu0];
			else % 3D
				C = zeros(6, 6);
				C(1:3, 1 : 3) = E0 / (1 + nu0) / (1 - 2 * nu0) * [1 - nu0 nu0 nu0;
				nu0 1 - nu0 nu0;
				nu0 nu0 1 - nu0];
				C(4:6, 4 : 6) = E0 / 2 / (1 + nu0) * eye(3);
			end
		*/

		mat3 C = mat3(0.0);

		if (stress_state == true) { // PLAIN STRAIN

			
			C[0][0] = 1.0 - nu;
			C[1][0] = nu;
			C[0][1] = nu;
			C[1][1] = 1.0 - nu;
			C[2][2] = 0.5 - nu;

			C = C * E / (1.0 + nu) / (1.0 - 2.0 * nu);
		}


		return C;
	}
}

double mpm::FixedCorotationalElasticity::EnergyDensity(mat2 Fe, double lam, double mew)
{
	mat2 R, S;
	PolarDecomp(Fe, R, S);

	mat2 U, V;
	double sig1, sig2;
	SVD(R, S, U, sig1, sig2, V);

	double J = glm::determinant(Fe);

	return mew * ((sig1 - 1.0)*(sig1 - 1.0) + (sig2 - 1.0)*(sig2 - 1.0)) + lam * 0.5 * (J - 1.0) * (J - 1.0);
}

mat2 mpm::FixedCorotationalElasticity::PKTensor(mat2 Fe, double lam, double mew)
{
	mat2 Fit = glm::transpose(glm::inverse(Fe));
	double J = glm::determinant(Fe);

	// calculate the polar decomposition F = RU.
	// Then calculate P using F and R
	mat2 R, S;
	PolarDecomp(Fe, R, S);

	mat2 P = 2.0 * mew * (Fe - R) + lam * (J - 1.0) * J * Fit;
	return P;
}

mat2 mpm::FixedCorotationalElasticity::dPdlam(mat2 Fe)
{
	mat2 Fit = glm::transpose(glm::inverse(Fe));
	double J = glm::determinant(Fe);

	return (J - 1.0) * J * Fit;
}

mat2 mpm::FixedCorotationalElasticity::dPdmew(mat2 Fe)
{
	mat2 R, S;
	PolarDecomp(Fe, R, S);

	return 2.0 * (Fe - R);
}

mat4 mpm::FixedCorotationalElasticity::d2Psi_dF2_Mat4(mat2 Fe, double lam, double mew)
{
	mat2 R, S, U, V;
	double s1, s2;
	PolarDecomp(Fe, R, S);
	SVD(R, S, U, s1, s2, V);

	double u1 = ExtractRotationAngle(U);
	double v1 = ExtractRotationAngle(V);

	// pooped out from MATLAB script: MATLAB/P_SVD.mlx
	// Assuming release mode will optimize this bad boy

	double cosu1 = cos(u1);
	double sinu1 = sin(u1);
	double cosv1 = cos(v1);
	double sinv1 = sin(v1);

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

mat2 mpm::FixedCorotationalElasticity::d2Psi_dF2_multbydF(mat2 Fe, double lam, double mew, mat2 dF)
{
	mat2 R, S;
	PolarDecomp(Fe, R, S);
	double J = glm::determinant(Fe);
	mat2 Fit = glm::transpose(glm::inverse(Fe));

	mat2 A = mat2(0.0);

	A += lam * J * Fit * InnerProduct(J * Fit, dF);

	A += lam * (J - 1.0) * CofactorMatrix(dF);

	A += 2.0 * mew * dF;

	double b = R[1][0] * dF[0][0] + R[1][1] * dF[0][1] - (dF[1][0] * R[0][0] + dF[1][1] * R[0][1]);
	double a = b / (S[0][0] + S[1][1]);
	mat2 dR = R * mat2(0, a, -a, 0); // column major

	A -= 2.0 * mew * dR;

	return A;
}

mat4 mpm::FixedCorotationalElasticity::d2Psi_dF2_Mat4_trick(mat2 Fe, double lam, double mew)
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
	mat2 dPdF00 = d2Psi_dF2_multbydF(Fe, lam, mew, m00);

	// NOTE: My math says these are the correct matrices, but when I run the code, the other way is correct
	// |0, 0|
	// |1, 0|
	mat2 dPdF01 = d2Psi_dF2_multbydF(Fe, lam, mew, m01);

	// |0, 1|
	// |0, 0|
	mat2 dPdF10 = d2Psi_dF2_multbydF(Fe, lam, mew, m10);


	// swap these around??? did I make a mistake somewhere
	//// |0, 0|
	//// |1, 0|
	//mat2 dPdF10 = d2Psi_dF2_multbydF(Fe, mew, lam, m01);

	//// |0, 1|
	//// |0, 0|
	//mat2 dPdF01 = d2Psi_dF2_multbydF(Fe, mew, lam, m10);

	// |0, 0|
	// |0, 1|
	mat2 dPdF11 = d2Psi_dF2_multbydF(Fe, lam, mew, m11);

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
	dPdF[0][1] = dPdF00[0][1];
	dPdF[0][2] = dPdF00[1][0];
	dPdF[0][3] = dPdF00[1][1];

	dPdF[1][0] = dPdF01[0][0];
	dPdF[1][1] = dPdF01[0][1];
	dPdF[1][2] = dPdF01[1][0];
	dPdF[1][3] = dPdF01[1][1];

	dPdF[2][0] = dPdF10[0][0];
	dPdF[2][1] = dPdF10[0][1];
	dPdF[2][2] = dPdF10[1][0];
	dPdF[2][3] = dPdF10[1][1];

	dPdF[3][0] = dPdF11[0][0];
	dPdF[3][1] = dPdF11[0][1];
	dPdF[3][2] = dPdF11[1][0];
	dPdF[3][3] = dPdF11[1][1];

	return dPdF;
}

mat4 mpm::FixedCorotationalElasticity::d2Psi_dF2_Mat4_fd(mat2 F, double lam, double mew)
{
	mat4 fddPdF;

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			double originalValue = F[i][j];

			F[i][j] = originalValue + 0.000001;
			mat2 P1 = FixedCorotationalElasticity::PKTensor(F, lam, mew);

			F[i][j] = originalValue - 0.000001;
			mat2 P2 = FixedCorotationalElasticity::PKTensor(F, lam, mew);

			mat2 dPdF_ij = (P1 - P2) / (2.0 * 0.000001);

			for (int a = 0; a < 2; a++) {
				for (int b = 0; b < 2; b++) {
					fddPdF[i * 2 + a][j * 2 + b] = dPdF_ij[a][b];
				}
			}
		}
	}
	return fddPdF;
}
