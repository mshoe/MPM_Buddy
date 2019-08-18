#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <vector>

struct MaterialPoint {
	glm::vec2 x;
	glm::vec2 v;
	float m;
	float vol; // initial volume
	glm::mat2 B;
	glm::mat2 F;
	glm::mat2 P;

	friend std::ostream & operator << (std::ostream &out, const MaterialPoint &c) {
		out << "x: " << glm::to_string(c.x) << "\n";
		out << "v: " << glm::to_string(c.v) << "\n";
		out << "m: " << c.m << "\n";
		out << "vol: " << c.vol << "\n";
		out << "B: " << glm::to_string(c.B) << "\n";
		out << "F: " << glm::to_string(c.F) << "\n";
		out << "P: " << glm::to_string(c.P);
		return out;
	}
};


// optimize move semantics later
struct PointCloud {
	PointCloud() {};
	~PointCloud() {
		glDeleteBuffers(1, &ssbo);
	};

	size_t N;
	std::vector<MaterialPoint> points;
	glm::vec3 color = glm::vec3(1.f, 0.f, 0.f);

	float mew;
	float lam;

	GLuint ssbo;
};