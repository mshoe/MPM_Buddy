#pragma once

#include "Shape.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <functional>
namespace sdf {

	typedef std::function<float(Shape, glm::vec2)> sdFunc;

	float circle(const glm::vec2& p, const glm::vec2& c, float r);
	float donut(const glm::vec2& p, const glm::vec2& c, float r1, float r2);
	float rectangle(const glm::vec2 &p, const glm::vec2 c, float w, float l);
	float DemoCircle(const Shape& shape, const glm::vec2& p);
	float DemoDonut(const Shape& shape, const glm::vec2& p);
	float DemoRectangle(const Shape& shape, const glm::vec2& p);
	
}