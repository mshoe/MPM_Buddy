#pragma once

#include "Constants.h"
#include <string>
#include <iomanip>
#include <sstream>

namespace glmToMATLAB {
	std::string MatStr(mat2 mat) {
		std::ostringstream streamObj;
		streamObj << std::setprecision(std::numeric_limits<double>::digits10 + 1);
		streamObj << "[" << mat[0][0] << ", " << mat[1][0] << "; " << mat[0][1] << ", " << mat[1][1] << "]";
		return streamObj.str();
	}
}