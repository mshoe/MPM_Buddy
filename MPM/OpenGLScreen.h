#pragma once
#include "Constants.h"

class OpenGLScreen {
public:
	OpenGLScreen();
	~OpenGLScreen();

	void Render();

	vec2 center;
	vec2 dimensions;

private:



	// For drawing the screen
	GLuint VAO, VBO, EBO;

	// For storing the texture of the object
	GLuint texture;
};