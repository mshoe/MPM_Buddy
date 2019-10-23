#include "glm_imgui.h"

void ImGui::DisplayNamedBoolColor(std::string name, bool value, glm::highp_fvec4 true_color, glm::highp_fvec4 false_color)
{
	std::string beginStr = name + " = ";
	ImGui::Text(beginStr.c_str());
	ImGui::SameLine();
	if (value) {
		ImVec4 imcolor = ImVec4(true_color.x, true_color.y, true_color.z, true_color.w);
		ImGui::TextColored(imcolor, "true");
	}
	else {
		ImVec4 imcolor = ImVec4(false_color.x, false_color.y, false_color.z, false_color.w);
		ImGui::TextColored(imcolor, "false");
	}
}

void ImGui::DisplayGlmRealColor(real value, glm::highp_fvec4 color)
{
	std::ostringstream numStream;
	numStream << std::setprecision(std::numeric_limits<real>::digits10 + 1);
	numStream << value;
	ImVec4 imcolor = ImVec4(color.x, color.y, color.z, color.w);
	ImGui::TextColored(imcolor, numStream.str().c_str());
}

void ImGui::DisplayNamedGlmRealColor(std::string name, real value, glm::highp_fvec4 color)
{
	std::string beginStr = name + " = [ ";
	ImGui::Text(beginStr.c_str());

	ImGui::PopupCopyRealMATLAB(name, value);

	ImGui::SameLine();
	DisplayGlmRealColor(value, color);
	ImGui::SameLine();
	ImGui::Text(" ]");
}

void ImGui::DisplayGlmRealMixColor(real value, real min_element, real max_element, glm::highp_fvec4 min_color, glm::highp_fvec4 max_color)
{
	std::ostringstream numStream;
	numStream << std::setprecision(std::numeric_limits<real>::digits10 + 1);
	numStream << value;
	real mix_value = glm::clamp((value - min_element) / (max_element - min_element), 0.0, 1.0);
	if (isnan(mix_value)) {
		mix_value = 0;
	}
	glm::highp_fvec4 color = glm::mix(min_color, max_color, mix_value);
	ImVec4 imcolor = ImVec4(color.x, color.y, color.z, color.w);
	ImGui::TextColored(imcolor, numStream.str().c_str());
}

void ImGui::DisplayGlmVec(vec2 vec)
{
	ImGui::Text("[ ");
	ImGui::SameLine();
	DisplayGlmRealColor(vec.x, glm::highp_fvec4(1.0));
	ImGui::SameLine();
	ImGui::Text(", ");
	ImGui::SameLine();
	DisplayGlmRealColor(vec.y, glm::highp_fvec4(1.0));
	ImGui::SameLine();
	ImGui::Text(" ]");
}

void ImGui::DisplayNamedGlmVecMixColor(std::string name, vec2 vec, glm::highp_fvec4 min_color, glm::highp_fvec4 max_color)
{
	std::string beginStr = name + " = [ ";
	ImGui::Text(beginStr.c_str());

	ImGui::PopupCopyVecMATLAB(name, vec);

	ImGui::SameLine();
	if (vec.x > vec.y) {
		DisplayGlmRealColor(vec.x, max_color);
		ImGui::SameLine();
		ImGui::Text(" , ");
		ImGui::SameLine();
		DisplayGlmRealColor(vec.y, min_color);
	}
	else {
		DisplayGlmRealColor(vec.x, min_color);
		ImGui::SameLine();
		ImGui::Text(" , ");
		ImGui::SameLine();
		DisplayGlmRealColor(vec.y, max_color);
	}
	ImGui::SameLine();
	ImGui::Text(" ]");
}

void ImGui::DisplayNamedGlmMatrixMixColor(std::string name, mat2 mat, glm::highp_fvec4 min_color, glm::highp_fvec4 max_color)
{

	real max_element = glm::max(glm::max(mat[0][0], mat[0][1]), glm::max(mat[1][0], mat[1][1]));
	real min_element = glm::min(glm::min(mat[0][0], mat[0][1]), glm::min(mat[1][0], mat[1][1]));	

	std::string firstLineStr = name + " = [ ";

	ImGui::Text(firstLineStr.c_str());

	ImGui::PopupCopyMatrixMATLAB(name, mat);
	
	ImGui::SameLine();

	DisplayGlmRealMixColor(mat[0][0], min_element, max_element, min_color, max_color);
	ImGui::SameLine();

	ImGui::Text(" , ");
	ImGui::SameLine();

	DisplayGlmRealMixColor(mat[1][0], min_element, max_element, min_color, max_color);
	ImGui::SameLine();

	ImGui::Text(" ]");

	char space = ' ';
	std::string secondLineStr = std::string(name.length(), space) + "   [ ";
	ImGui::Text(secondLineStr.c_str());
	ImGui::SameLine();

	DisplayGlmRealMixColor(mat[0][1], min_element, max_element, min_color, max_color);
	ImGui::SameLine();

	ImGui::Text(" , ");
	ImGui::SameLine();

	DisplayGlmRealMixColor(mat[1][1], min_element, max_element, min_color, max_color);
	ImGui::SameLine();

	ImGui::Text(" ]");
}

void ImGui::PopupCopyRealMATLAB(std::string name, real value)
{
	std::string menu_str = name + " menu";
	std::string copy_str = "Copy " + name + " (MATLAB)";
	if (ImGui::BeginPopupContextItem(menu_str.c_str())) {
		if (ImGui::Button(copy_str.c_str())) {
			ImGui::SetClipboardText(glmToMATLAB::RealStr(value).c_str());
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}

void ImGui::PopupCopyVecMATLAB(std::string name, vec2 vec)
{
	std::string menu_str = name + " menu";
	std::string copy_str = "Copy " + name + " (MATLAB)";
	if (ImGui::BeginPopupContextItem(menu_str.c_str())) {
		if (ImGui::Button(copy_str.c_str())) {
			ImGui::SetClipboardText(glmToMATLAB::VecStr(vec).c_str());
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}

void ImGui::PopupCopyMatrixMATLAB(std::string name, mat2 mat)
{
	std::string menu_str = name + " menu";
	std::string copy_str = "Copy " + name + " (MATLAB)";
	if (ImGui::BeginPopupContextItem(menu_str.c_str())) {
		if (ImGui::Button(copy_str.c_str())) {
			ImGui::SetClipboardText(glmToMATLAB::MatStr(mat).c_str());
			ImGui::CloseCurrentPopup();
		}
		ImGui::EndPopup();
	}
}
