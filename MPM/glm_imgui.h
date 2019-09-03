#pragma once
#include "Constants.h"

#include "glm_MATLAB.h"
#include <string>
#include <iomanip>
#include <sstream>
namespace ImGui {

	void DisplayNamedBoolColor(std::string name, bool value, glm::highp_fvec4 true_color, glm::highp_fvec4 false_color);
	void DisplayGlmRealColor(real value, glm::highp_fvec4 color);
	void DisplayNamedGlmRealColor(std::string name, real value, glm::highp_fvec4 color);
	void DisplayGlmRealMixColor(real value, real min_element, real max_element, glm::highp_fvec4 min_color, glm::highp_fvec4 max_color);
	void DisplayNamedGlmVecMixColor(std::string name, vec2 vec, glm::highp_fvec4 min_color, glm::highp_fvec4 max_color);
	void DisplayNamedGlmMatrixMixColor(std::string name, mat2 mat, glm::highp_fvec4 min_color, glm::highp_fvec4 max_color);
	void PopupCopyRealMATLAB(std::string name, real value);
	void PopupCopyVecMATLAB(std::string name, vec2 vec);
	void PopupCopyMatrixMATLAB(std::string name, mat2 mat);
}