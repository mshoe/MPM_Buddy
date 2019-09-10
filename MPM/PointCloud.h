#pragma once

#include "Constants.h"

#include <glad/glad.h>
#include <vector>
#include <iomanip>

enum MATERIAL_PARAMETERS_ENUM {
	NEO_HOOKEAN_ELASTICITY = 0,
	FIXED_COROTATIONAL_ELASTICITY = 1,
	SIMPLE_SNOW = 2
};

struct MaterialPoint {
	vec2 x;
	vec2 v;
	GLreal m;
	GLreal vol; // initial volume
	mat2 B;
	mat2 Fe;
	mat2 Fp;
	mat2 P;

	// extra not neccessary to store, but useful for debugging:
	mat2 FePolar_R;
	mat2 FePolar_S;
	mat2 FeSVD_U;
	mat2 FeSVD_S;
	mat2 FeSVD_V;

	real energy = 0.0;
	real opengl_padding2 = 0.696942080086969;

	friend std::ostream & operator << (std::ostream &out, const MaterialPoint &c) {
		out << std::setprecision(std::numeric_limits<double>::digits10 + 1);
		out << "x: " << glm::to_string(c.x) << "\n";
		out << "v: " << glm::to_string(c.v) << "\n";
		out << "m: " << c.m << "\n";
		out << "vol: " << c.vol << "\n";
		out << "B: " << glm::to_string(c.B) << "\n";
		out << "Fe: " << glm::to_string(c.Fe) << "\n";
		out << "Fe (highp): " << "(" << c.Fe[0][0] << ", " << c.Fe[0][1] << "), (" << c.Fe[1][0] << ", " << c.Fe[1][1] << ")" << "\n";
		out << "Fp: " << glm::to_string(c.Fp) << "\n";
		out << "P: " << glm::to_string(c.P) << "\n";
		out << "FePolar_R: " << glm::to_string(c.FePolar_R) << "\n";
		out << "FePolar_S: " << glm::to_string(c.FePolar_S) << "\n";
		out << "FeSVD_U: " << glm::to_string(c.FeSVD_U) << "\n";
		out << "FeSVD_S: " << glm::to_string(c.FeSVD_S) << "\n";
		out << "FeSVD_V: " << glm::to_string(c.FeSVD_V) << "\n";
		out << "energy: " << c.energy << "\n";
		out << "opengl_padding2: " << c.opengl_padding2 << "\n";
		return out;
	}
};

struct MaterialParameters {
	real youngMod = 400.0;
	real poisson = 0.3;


	real particleSpacing = 0.25;
	real density = 0.16;

	real crit_c = 0.025;
	real crit_s = 0.0075;
	real hardening = 10.0;
};

// optimize move semantics later
struct PointCloud {
	PointCloud() {};
	~PointCloud() {
		glDeleteBuffers(1, &ssbo);
	};

	size_t N;
	std::vector<MaterialPoint> points;
	glm::highp_fvec4 color = glm::highp_fvec4(1.f, 0.f, 0.f, 1.f);

	MaterialParameters parameters;

	real mew;
	real lam;

	GLuint comodel = 1;

	GLuint ssbo;
};