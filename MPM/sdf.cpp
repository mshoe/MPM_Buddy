#include "sdf.h"

float sdf::circle(const glm::vec2& p, const glm::vec2& c, float r) {
	return glm::length(p - c) - r;
}

float sdf::donut(const glm::vec2& p, const glm::vec2& c, float r1, float r2) {
	return glm::max(-circle(p, c, r1), circle(p, c, r2));
}

float sdf::rectangle(const glm::vec2 & p, const glm::vec2 c, float w, float l)
{
	return 0.0f;
}

float sdf::DemoCircle(const Shape& shape, const glm::vec2& p) {
	if (shape.radii.empty()) {
		std::cout << "Error in sdf::DemoCircle. shape.radii is empty.\n";
		return -1.f;
	}
	return circle(p, shape.pos, shape.radii[0]);
}

float sdf::DemoDonut(const Shape & shape, const glm::vec2 & p)
{
	if (shape.radii.size() < 2) {
		std::cout << "Error in sdf::DemoDonut. Not enough radii in shape.\n";
		return -1.f;
	}
	return donut(p, shape.pos, shape.radii[0], shape.radii[1]);
}

float sdf::DemoRectangle(const Shape & shape, const glm::vec2 & p)
{
	return 0.0f;
}
