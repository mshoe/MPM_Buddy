#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glad/glad.h>
#include "imgui/imgui.h"

#define USE_DOUBLE 1

#ifdef USE_DOUBLE
typedef double real;
typedef glm::highp_dmat2 mat2;
typedef glm::highp_dmat3 mat3;
typedef glm::highp_dmat4 mat4;
typedef glm::highp_dvec1 vec1;
typedef glm::highp_dvec2 vec2;
typedef glm::highp_dvec3 vec3;
typedef glm::highp_dvec4 vec4;
typedef GLdouble GLreal;
namespace ImGui {
	bool InputReal(const char* label, real* v, real step = 0.0, real step_fast = 0.0, const char* format = "%.6f", ImGuiInputTextFlags flags = 0);
}
#else
#define real float;
#endif

const int SRC_WIDTH = 1800;
const int SRC_HEIGHT = 900;
const real S_PER_UPDATE = 0.01666666666666667;

const int CHUNK_WIDTH = 32; // chunks are square

const int GRID_SIZE_X = CHUNK_WIDTH * 4;
const int GRID_SIZE_Y = CHUNK_WIDTH * 4;

const int G_NUM_GROUPS_X = 4;
const int G_NUM_GROUPS_Y = 4;
const int G2P_WORKGROUP_SIZE = 1024;