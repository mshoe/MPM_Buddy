#pragma once
#include "Constants.h"
#include "MpmFunctions.h"

namespace mpm {
	mat2 NeoHookeanPKTensor(mat2 Fe, real lam, real mew);

	

	mat2 SimpleSnowPKTensor(mat2& Fe, mat2& Fp, real lam, real mew, real crit_c, real crit_s, real hardening);

	namespace FixedCorotationalElasticity {
		mat2 PKTensor(mat2 Fe, real lam, real mew);
		mat4 d2Psi_dF2_Mat4(mat2 Fe, real lam, real mew); // don't use this, not working


		mat2 d2Psi_dF2_multbydF(mat2 Fe, real lam, real mew, mat2 dF);
		mat4 d2Psi_dF2_Mat4_trick(mat2 Fe, real lam, real mew);
	}
	
}
