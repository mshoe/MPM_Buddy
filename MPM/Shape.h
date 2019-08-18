#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <iostream>
struct Shape {
	Shape() {};
	Shape(glm::vec2 _pos, std::vector<float> _radii) :
		pos(_pos),
		radii(_radii) {}
	~Shape() {};

	glm::vec2 pos;
	std::vector<float> radii;
};