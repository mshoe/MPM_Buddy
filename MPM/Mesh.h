#pragma once

#include "Constants.h"
#include "Shader.h"

#include <vector>

class Mesh {
public:
	std::vector<vec2>				vertices;
	std::vector<unsigned int>		indices;

	std::vector<float>				oglVerts;

	
	Mesh(std::vector<vec2> verts, std::vector<unsigned int> inds);

	
	void Draw();

private:
	unsigned int VAO, VBO, EBO;

	void setupMesh();
};