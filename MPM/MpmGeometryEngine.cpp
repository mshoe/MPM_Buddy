#include "MpmGeometryEngine.h"

void mpm::MpmGeometryEngine::ProcessMouseInput()
{
	
}

void mpm::MpmGeometryEngine::ProcessKeyboardInput(GLFWwindow* window, real lag)
{
	if (m_mpmAlgorithmEngine->m_paused) {

		if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
			ClearCreateStates();
			m_createCircleState = true;
		}

		if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
			ClearCreateStates();
			m_createRectState = true;
		}

		if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
			ClearCreateStates();
			m_createIsoTriState = true;
		}

		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
			m_addPolygonVertexState = true;
		}

		if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) {
			m_addPWLineVertexState = true;
		}
	}
}