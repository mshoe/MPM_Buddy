#include "OpenGLScreen.h"
#include <iostream>

OpenGLScreen::OpenGLScreen()
{
	InitOpenGLScreen();
}

OpenGLScreen::~OpenGLScreen()
{
	CleanupOpenGLScreen();
}

void OpenGLScreen::Render()
{
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void OpenGLScreen::InitOpenGLScreen()
{
	// Create screen on the right half of window
// ** VERTICES need to be float in glsl ** //

	// modify the vertices in the vertex shader
	float vertices[] = {
		1.f,  1.f, // top right
		1.f, -1.f, // bottom right
		-1.f, -1.f, // bottom left
		-1.f,  1.f, // top left 
	};
	unsigned int indices[] = {
		0, 1, 3,   // first triangle
		1, 2, 3    // second triangle
	};


	// ..:: Initialization code :: ..
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);
	glCreateVertexArrays(1, &VAO);
	glCreateBuffers(1, &VBO);
	glCreateBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glNamedBufferStorage(VBO, sizeof(vertices), NULL, GL_DYNAMIC_STORAGE_BIT);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glNamedBufferSubData(VBO, 0, sizeof(vertices), vertices);

	glNamedBufferStorage(EBO, sizeof(indices), NULL, GL_DYNAMIC_STORAGE_BIT);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glNamedBufferSubData(EBO, 0, sizeof(indices), indices);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

void OpenGLScreen::CleanupOpenGLScreen()
{
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);
}

ImGuiScreen::ImGuiScreen(vec2 screen_dimensions)
{

	InitOpenGLScreen();

	this->screen_dimensions = screen_dimensions;
	
	// modify the vertices in the vertex shader
	float vertices[] = {
		1.f,  1.f, // top right
		1.f, -1.f, // bottom right
		-1.f, -1.f, // bottom left
		-1.f,  1.f, // top left 
	};
	unsigned int indices[] = { // draw with GL_LINE_LOOP
		0, 1, 2, 3 
	};


	// ..:: Initialization code :: ..
	glDeleteVertexArrays(1, &borderVAO);
	glDeleteBuffers(1, &borderVBO);
	glDeleteBuffers(1, &borderEBO);
	glCreateVertexArrays(1, &borderVAO);
	glCreateBuffers(1, &borderVBO);
	glCreateBuffers(1, &borderEBO);

	glBindVertexArray(borderVAO);

	glNamedBufferStorage(borderVBO, sizeof(vertices), NULL, GL_DYNAMIC_STORAGE_BIT);
	glBindBuffer(GL_ARRAY_BUFFER, borderVBO);
	glNamedBufferSubData(borderVBO, 0, sizeof(vertices), vertices);

	glNamedBufferStorage(borderEBO, sizeof(indices), NULL, GL_DYNAMIC_STORAGE_BIT);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, borderEBO);
	glNamedBufferSubData(borderEBO, 0, sizeof(indices), indices);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	InitFBOAndTexture();
}

ImGuiScreen::~ImGuiScreen()
{
	CleanupOpenGLScreen();

	glDeleteVertexArrays(1, &borderVAO);
	glDeleteBuffers(1, &borderVBO);
	glDeleteBuffers(1, &borderEBO);

	glDeleteBuffers(1, &FBO);
}

void ImGuiScreen::BindFBO()
{
	glBindFramebuffer(GL_FRAMEBUFFER, FBO);
}

void ImGuiScreen::UnbindFBO()
{
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void ImGuiScreen::RenderLineLoop()
{
	glBindVertexArray(borderVAO);
	glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void ImGuiScreen::InitFBOAndTexture()
{
	// Create a framebuffer object
	glCreateFramebuffers(1, &FBO);

	// Create a texture for our color buffer
	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);
	glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, (GLsizei)screen_dimensions.x, (GLsizei)screen_dimensions.y);

	// Turning off mipmaps
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Attach color texture to FBO
	glNamedFramebufferTexture(FBO, GL_COLOR_ATTACHMENT0, texture, 0);

	// I think we don't need a depth buffer since this app is 2D...

	// Tell OpenGL that we want to draw into the FBO's only color attachment
	static const GLenum draw_buffers[] = { GL_COLOR_ATTACHMENT0 };
	glNamedFramebufferDrawBuffers(FBO, 1, draw_buffers);

	// Unbind texture
	glBindTexture(GL_TEXTURE_2D, 0);


	GLenum fboStatus = glCheckNamedFramebufferStatus(FBO, GL_DRAW_FRAMEBUFFER);
	if (fboStatus == GL_FRAMEBUFFER_COMPLETE) {
		std::cout << "Framebuffer complete." << std::endl;
	}
	else {
		std::cout << "Framebuffer incomplete" << std::endl;
	}
}
