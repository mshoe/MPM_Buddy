#pragma once
#include "Constants.h"
#include "MpmFunctions.h"

namespace mpm {

	enum class ENERGY_MODEL {
		LINEAR_ELASTICITY = 0,
		NEO_HOOKEAN_ELASTICITY = 1,
		FIXED_COROTATIONAL_ELASTICITY = 2,
		SIMPLE_SNOW = 3,
		Count
	};

	namespace LinearElasticity {
		double EnergyDensity(mat2 Fe, double lam, double mew);
		mat2 PKTensor(mat2 Fe, double lam, double mew);

		mat3 ElasticityMatrix(double E, double nu, bool stress_state);
	}

	namespace NeoHookean {
		mat2 PKTensor(mat2 Fe, double lam, double mew);
	}

	namespace FixedCorotationalElasticity {
		double EnergyDensity(mat2 Fe, double lam, double mew);
		mat2 PKTensor(mat2 Fe, double lam, double mew);
		mat2 dPdlam(mat2 Fe);
		mat2 dPdmew(mat2 Fe);

		mat4 d2Psi_dF2_Mat4(mat2 Fe, double lam, double mew); // don't use this, not working


		mat2 d2Psi_dF2_multbydF(mat2 Fe, double lam, double mew, mat2 dF);
		mat4 d2Psi_dF2_Mat4_trick(mat2 Fe, double lam, double mew);

		mat4 d2Psi_dF2_Mat4_fd(mat2 F, double lam, double mew);
	}
	
	namespace SimpleSnow {
		mat2 PKTensor(mat2& Fe, mat2& Fp, double lam, double mew, double crit_c, double crit_s, double hardening);
	}

	

	

}
