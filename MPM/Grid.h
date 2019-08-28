#pragma once

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>
#include <iostream>

struct GridNode {
	GridNode() {}
	GLfloat m = 0.f;

	// We need this padding here because when reading from buffers, they will put a byte in-between m and v.
	// This will offset out GridNode values.
	// With a padding, we can read GridNode structs easily.
	GLfloat glsl_padding = 0.f;

	glm::vec2 v = glm::vec2(0.0097f);
	glm::vec2 momentum = glm::vec2(0.0069f);
	glm::vec2 force = glm::vec2(0.0042f);
	
	glm::vec2 deltaForce = glm::vec2(0.69420f);
	glm::vec2 xk = glm::vec2(0.1234f);
	glm::vec2 rk = glm::vec2(0.4321f);
	glm::vec2 pk = glm::vec2(0.1010f);
	glm::vec2 Ark = glm::vec2(2.f);
	glm::vec2 Apk = glm::vec2(3.f);

	GLfloat rkArk = 0.1f;
	GLfloat glsl_padding2 = 0.f;

	friend std::ostream & operator << (std::ostream &out, const GridNode &c) {
		out << "m: " << c.m << "\n";
		out << "padding: " << c.glsl_padding << "\n";
		out << "v: " << glm::to_string(c.v) << "\n";
		out << "mv: " << glm::to_string(c.momentum) << "\n";
		out << "f: " << glm::to_string(c.force) << "\n";
		out << "df: " << glm::to_string(c.deltaForce) << "\n";
		out << "xk: " << glm::to_string(c.xk) << "\n";
		out << "rk: " << glm::to_string(c.rk) << "\n";
		out << "pk: " << glm::to_string(c.pk) << "\n";
		out << "Ark: " << glm::to_string(c.Ark) << "\n";
		out << "Apk: " << glm::to_string(c.Apk) << "\n";
		out << "rkArk: " << c.rkArk << "\n";
		out << "padding2: " << c.glsl_padding2 << "\n";
		return out;
	}
};

class Grid {
public:
	Grid() {
		nodes = std::vector<GridNode>(GRID_SIZE_X*GRID_SIZE_Y, GridNode());
	}
	Grid(size_t grid_size_x, size_t grid_size_y) {
		nodes = std::vector<GridNode>(grid_size_x*grid_size_y, GridNode());
	}
	~Grid() {}

	std::vector<GridNode> nodes;

	
};