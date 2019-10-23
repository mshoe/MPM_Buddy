#pragma once

#include "Constants.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>
#include <iostream>

struct GridNode {
	GridNode() {}
	GLreal m = 0.0;

	// We need this padding here because when reading from buffers, they will put a byte in-between m and v.
	// This will offset out GridNode values.
	// With a padding, we can read GridNode structs easily.
	GLreal glsl_padding = 0.0;

	vec2 v = vec2(0.0);
	vec2 momentum = vec2(0.0);
	vec2 force = vec2(0.0);
	vec2 nodalAcceleration = vec2(0.0);

	vec2 deltaForce = vec2(0.0);
	vec2 xk = vec2(0.0);
	vec2 rk = vec2(0.0);
	vec2 pk = vec2(0.0);
	vec2 Ark = vec2(0.0);
	vec2 Apk = vec2(0.0);

	GLreal rkArk = 0.0;
	//GLreal glsl_padding2 = 0.0;
	bool converged = true;
	bool packing; // Packing becuz C++ stores bool as 1 byte, but GLSL is reading/writing bool every 4 bytes
	bool packing2;
	bool packing3;

	bool selected = false;


	friend std::ostream & operator << (std::ostream &out, const GridNode &c) {
		out << "m: " << c.m << "\n";
		out << "padding: " << c.glsl_padding << "\n";
		out << "v: " << glm::to_string(c.v) << "\n";
		out << "mv: " << glm::to_string(c.momentum) << "\n";
		out << "f: " << glm::to_string(c.force) << "\n";
		out << "nodal acc: " << glm::to_string(c.nodalAcceleration) << "\n";
		out << "df: " << glm::to_string(c.deltaForce) << "\n";
		out << "xk: " << glm::to_string(c.xk) << "\n";
		out << "rk: " << glm::to_string(c.rk) << "\n";
		out << "pk: " << glm::to_string(c.pk) << "\n";
		out << "Ark: " << glm::to_string(c.Ark) << "\n";
		out << "Apk: " << glm::to_string(c.Apk) << "\n";
		out << "rkArk: " << c.rkArk << "\n";
		out << "converged: " << c.converged << "\n";
		out << "selected: " << c.selected << "\n";
		//out << "padding2: " << c.glsl_padding2 << "\n";
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