#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <memory>

#include "Shader.h"

class Engine {
public:
	//Engine() {}
	//~Engine() { CleanupComputeShaderPipeline(); };

	virtual bool InitComputeShaderPipeline() = 0;
	virtual bool CleanupComputeShaderPipeline() = 0;

	virtual void Render() = 0;
	virtual void RenderGUI() = 0;

	void SetMouseValues(glm::vec2 _mousePos, bool _leftButtonDown, bool _rightButtonDown) {
		m_mousePos = _mousePos;
		m_leftButtonDown = _leftButtonDown;
		m_rightButtonDown = _rightButtonDown;
	}


	std::shared_ptr<StandardShader> GetMouseShader() const { return m_mouseShader; }
protected:
	std::shared_ptr<StandardShader> m_mouseShader = nullptr;

	glm::vec2 m_mousePos = glm::vec2(0.0, 0.0);
	bool m_leftButtonDown = false;
	bool m_rightButtonDown = false;
};