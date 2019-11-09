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
			MpmReset_GLSL();
		}
	}
}



void mpm::MpmAlgorithmEngine::MapCPUPointCloudsToGPU()
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {
		void* ptr = glMapNamedBuffer(pointCloudPair.second->ssbo, GL_WRITE_ONLY);
		MaterialPoint* data = static_cast<MaterialPoint*>(ptr);
		memcpy(data, pointCloudPair.second->points.data(), pointCloudPair.second->points.size() * sizeof(MaterialPoint));
		glUnmapNamedBuffer(pointCloudPair.second->ssbo);
	}
}

void mpm::MpmAlgorithmEngine::MapCPUGridToGPU()
{
	void* ptr = glMapNamedBuffer(m_mpmEngine->gridSSBO, GL_WRITE_ONLY);
	GridNode* data = static_cast<GridNode*>(ptr);
	memcpy(data, m_mpmEngine->m_grid.nodes.data(), m_mpmEngine->m_grid.nodes.size() * sizeof(GridNode));
	glUnmapNamedBuffer(m_mpmEngine->gridSSBO);
}