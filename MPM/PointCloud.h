#pragma once

#include "Constants.h"

#include <glad/glad.h>
#include <vector>

struct MaterialPoint {
	glm::vec2 x;
	glm::vec2 v;
	float m;
	float vol; // initial volume
	glm::mat2 B;
	glm::mat2 Fe;
	glm::mat2 Fp;
	glm::mat2 P;

	// extra not neccessary to store, but useful for debugging:
	glm::mat2 FeSVD_U;
	glm::mat2 FeSVD_S;
	glm::mat2 FeSVD_V;

	friend std::ostream & operator << (std::ostream &out, const MaterialPoint &c) {
		out << "x: " << glm::to_string(c.x) << "\n";
		out << "v: " << glm::to_string(c.v) << "\n";
		out << "m: " << c.m << "\n";
		out << "vol: " << c.vol << "\n";
		out << "B: " << glm::to_string(c.B) << "\n";
		out << "Fe: " << glm::to_string(c.Fe) << "\n";
		out << "Fp: " << glm::to_string(c.Fp) << "\n";
		out << "P: " << glm::to_string(c.P) << "\n";
		out << "FeSVD_U: " << glm::to_string(c.FeSVD_U) << "\n";
		out << "FeSVD_S: " << glm::to_string(c.FeSVD_S) << "\n";
		out << "FeSVD_V: " << glm::to_string(c.FeSVD_V) << "\n";
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
	glm::vec4 color = glm::vec4(1.f, 0.f, 0.f, 1.f);

	float mew;
	float lam;

	float crit_c;
	float crit_s;
	float hardening;
	GLuint comodel = 1;

	GLuint ssbo;
};