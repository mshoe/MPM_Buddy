#include "MpmAlgorithmEngine.h"

void mpm::MpmAlgorithmEngine::ProcessMouseInput()
{
}

void mpm::MpmAlgorithmEngine::ProcessKeyboardInput(GLFWwindow* window, real lag)
{
	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
		m_paused = true;

	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
		if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
			m_paused = false;
		}
	}



	if (m_paused) {
		if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS) {
			MpmReset();
		}
	}
}



