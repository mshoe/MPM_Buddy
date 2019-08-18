#pragma once

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "Constants.h"
#include "Shader.h"
#include "Camera.h"
#include "Engine.h"
#include "MpmEngine.h"
#include "ModelingEngine.h"

#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <memory>
#include <iostream>
#include <chrono>

class MainEngine
{

private:
	bool Init();
	bool Cleanup();

public:
	MainEngine();
	~MainEngine();

	bool InitShaderPipeline();
	bool InitScreen();
	bool InitMPM();

	void Loop();
	void ProcessInput(GLFWwindow *window, float lag);
	void Update(float lag);
	void Render();

private:
	GLFWwindow *m_window;
	bool m_cameraImgui = true;

	std::shared_ptr<StandardShader> m_mouseShader;
	std::unique_ptr<Camera> m_camera;
	std::shared_ptr<mpm::MpmEngine> m_mpmEngine;

	std::shared_ptr<Engine> m_currentEngine;

	float m_time = 0.f;

	// Rendering stuff
	GLuint VAO;
	GLuint VBO;
	GLuint EBO;
};

