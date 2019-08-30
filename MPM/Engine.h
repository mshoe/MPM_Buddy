#pragma once
#include "Constants.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

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

	void SetMouseValues(vec2 _mousePos, bool _leftButtonDown, bool _rightButtonDown) {
		m_mousePos = _mousePos;
		m_leftButtonDown = _leftButtonDown;
		m_rightButtonDown = _rightButtonDown;
	}

	std::shared_ptr<StandardShader> GetMouseShader() const { return m_mouseShader; }
protected:
	std::shared_ptr<StandardShader> m_mouseShader = nullptr;

	vec2 m_mousePos = vec2(0.0, 0.0);
	bool m_leftButtonDown = false;
	bool m_rightButtonDown = false;
};