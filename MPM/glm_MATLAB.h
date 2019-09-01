#pragma once

#include "Constants.h"
#include <string>
#include <iomanip>
#include <sstream>

namespace glmToMATLAB {
	std::string MatStr(mat2 mat);
	std::string VecStr(vec2 vec);
	std::string RealStr(real value);
}