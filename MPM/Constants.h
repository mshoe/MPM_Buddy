#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

#define USE_DOUBLE 1

#ifdef USE_DOUBLE
#define real double;
#else
#define real float;
#endif

const int SRC_WIDTH = 1800;
const int SRC_HEIGHT = 900;
const float S_PER_UPDATE = 0.016667f;

const int GRID_SIZE_X = 32 * 4;
const int GRID_SIZE_Y = 32 * 4;

const int G_NUM_GROUPS_X = 4;
const int G_NUM_GROUPS_Y = 4;
const int G2P_WORKGROUP_SIZE = 1024;