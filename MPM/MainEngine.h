#pragma once

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "Constants.h"
#include "Shader.h"
#include "Camera.h"
#include "Engine.h"
#include "MpmEngine.h"

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

	void Loop();
	void ProcessInput(GLFWwindow *window, real lag);
	void Render();

private:
	GLFWwindow *m_window;

	std::shared_ptr<mpm::MpmEngine> m_mpmEngine;

	real m_time = 0.f;
};

