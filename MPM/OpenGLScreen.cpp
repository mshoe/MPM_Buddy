#include "OpenGLScreen.h"

OpenGLScreen::OpenGLScreen()
{
	// Create screen on the right half of window
	// ** VERTICES need to be float in glsl ** //
	float vertices[] = {
		1.f,  1.f, // top right
		1.f, -1.f, // bottom right
		0.f, -1.f, // bottom left
		0.f,  1.f, // top left 
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

OpenGLScreen::~OpenGLScreen()
{
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);
}

void OpenGLScreen::Render()
{
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}
