#pragma once
#include "Constants.h"
#include "MpmFunctions.h"

namespace mpm {
	mat2 NeoHookeanPKTensor(mat2 Fe, real lam, real mew);

	

	mat2 SimpleSnowPKTensor(mat2& Fe, mat2& Fp, real mew, real lam, real crit_c, real crit_s, real hardening);

	namespace FixedCorotationalElasticity {
		mat2 PKTensor(mat2 Fe, real lam, real mew);
		mat4 d2Psi_dF2_Mat4(double mew, double lam, mat2 Fe);
	}
	
}
