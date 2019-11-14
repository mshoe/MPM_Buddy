#include "glm_MATLAB.h"

std::string glmToMATLAB::MatStr(mat2 mat)
{
	std::ostringstream streamObj;
	streamObj << std::setprecision(std::numeric_limits<real>::digits10 + 1);
	streamObj << "[" << mat[0][0] << ", " << mat[1][0] << "; " << mat[0][1] << ", " << mat[1][1] << "]";
	return streamObj.str();
}

std::string glmToMATLAB::MatStr(mat4 mat)
{
	std::ostringstream streamObj;
	streamObj << std::setprecision(std::numeric_limits<real>::digits10 + 1);
	streamObj << "[";
	for (int i = 0; i < 4; i++) {
		streamObj << mat[0][i] << ", " << mat[1][i] << ", " << mat[2][i] << ", " << mat[3][i];
		if (i < 3)
			streamObj << "; ";
	}
	streamObj << "]";
	return streamObj.str();
}

std::string glmToMATLAB::VecStr(vec2 vec)
{
	std::ostringstream streamObj;
	streamObj << std::setprecision(std::numeric_limits<real>::digits10 + 1);
	streamObj << "[" << vec.x << "; " << vec.y << "]";
	return streamObj.str();
}

std::string glmToMATLAB::RealStr(real value)
{
	std::ostringstream streamObj;
	streamObj << std::setprecision(std::numeric_limits<real>::digits10 + 1);
	streamObj << value;
	return streamObj.str();
}
