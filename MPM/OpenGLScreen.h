#pragma once
#include "Constants.h"

class OpenGLScreen {
public:
	OpenGLScreen();
	virtual ~OpenGLScreen();

	void Render();

	// default is right half of the screen
	vec2 center = vec2(1350.0, 450.0);
	vec2 screen_dimensions = vec2(1800.0, 900.0);
	vec2 sim_dimensions = vec2(900.0, 900.0);

protected:
	void InitOpenGLScreen();
	void CleanupOpenGLScreen();

	// For drawing the screen
	GLuint VAO, VBO, EBO;
};

class ImGuiScreen : public OpenGLScreen {
public:
	ImGuiScreen(vec2 screen_dimensions);
	~ImGuiScreen();

	void BindFBO();
	void UnbindFBO();

	void RenderLineLoop();


	// For storing the texture of the object
	GLuint texture;

private:

	void InitFBOAndTexture();

	// framebuffer
	GLuint FBO;

	
};