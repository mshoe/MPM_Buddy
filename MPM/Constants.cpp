#include "Constants.h"

bool ImGui::InputReal(const char* label, real* v, real step, real step_fast, const char* format, ImGuiInputTextFlags flags)
{
	return InputDouble(label, v, step, step_fast, format, flags);
}
