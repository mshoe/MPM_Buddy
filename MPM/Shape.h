#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <iostream>

namespace sdf {
	struct Shape {
	public:

		virtual float Sdf(glm::vec2 p) = 0;

		glm::vec2 pos;
	};

	struct Circle : public Shape {
	private:
		Circle() {}

	public:
		Circle(glm::vec2 _pos, float _r) {
			pos = _pos;
			r = _r;
		}

		float Sdf(glm::vec2 p) {
			return glm::length(p - pos) - r;
		}

		float r;
	};

	struct Rectangle : public Shape {
	private:
		Rectangle() {}
	public:
		Rectangle(glm::vec2 _pos, float _b, float _h) {
			pos = _pos;
			b = _b;
			h = _h;
		}

		float Sdf(glm::vec2 p) {
			return 0.f;
		}

		float b, h; // b = horizontal base length, h = vertical height length;
	};

	struct BooleanShape {
	public:
		BooleanShape() {}

		//std::vector<Shape> m_shapeList;
		//std::vector<float


	};
}