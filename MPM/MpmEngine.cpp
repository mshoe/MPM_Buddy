#include "MpmEngine.h"

void mpm::MpmEngine::Update()
{
	m_mpmAlgorithmEngine->Update();
}

void mpm::MpmEngine::ProcessKeyboardInput(GLFWwindow* window, real lag)
{
	// don't process keyboard input when writing into imgui textboxes
	if (ImGui::GetIO().WantTextInput) {
		return;
	}

	


	if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
		m_zoomState = true;
	}

	

	/*if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) {
		for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
			SetDeformationGradients(pointCloudPair.first, mat2(1.0), mat2(1.0), false);
		}
	}*/

	m_mpmGeometryEngine->ProcessKeyboardInput(window, lag);
	m_mpmAlgorithmEngine->ProcessKeyboardInput(window, lag);


	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS) {
		m_selectNodeState = true;
	}

	if (m_movingZoomWindow) {
		if (glfwGetKey(window, GLFW_KEY_RIGHT)) {
			m_zoomPoint.x += 1.0;
		}
		if (glfwGetKey(window, GLFW_KEY_LEFT)) {
			m_zoomPoint.x -= 1.0;
		}
		if (glfwGetKey(window, GLFW_KEY_UP)) {
			m_zoomPoint.y += 1.0;
		}
		if (glfwGetKey(window, GLFW_KEY_DOWN)) {
			m_zoomPoint.y -= 1.0;
		}
	}

}

void mpm::MpmEngine::ProcessMouseInput(GLFWwindow* window, real lag)
{
	real xpos, ypos, left_click, right_click;
	glfwGetCursorPos(window, &xpos, &ypos);
	left_click = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) ? 1.0 : 0.0;
	right_click = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) ? 1.0 : 0.0;
	m_midButtonDown = (bool)glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE);

	if (m_mouseGlobalScreen.x != xpos || m_mouseGlobalScreen.y != (real)SRC_HEIGHT - ypos) {
		m_mouseMoved = true;
	}
	else {
		m_mouseMoved = false;
	}

	m_mouseGlobalScreen.x = xpos;
	m_mouseGlobalScreen.y = (real)SRC_HEIGHT - ypos;

	//std::cout << xpos << ", " << ypos << std::endl;

	// goal: Map mouse click to normalized grid space, which is (0.0, 1.0) when inside the MPM grid.
	// Grid space can be found from the OpenGLScreen's center and dimensions
	m_leftButtonDown = (bool)left_click;
	m_rightButtonDown = (bool)right_click;

	m_mpmGeometryEngine->ProcessMouseInput();
}

void mpm::MpmEngine::HandleStates()
{
	

	if (m_zoomState && m_leftButtonDown) {
		m_zoomState = false;
		m_zoomFactor += 0.5;
		m_zoomPoint = m_mouseMpmRenderScreenGridSpace;
	}

	if (m_zoomState && m_midButtonDown) {
		m_zoomState = false;
		m_zoomPoint = m_mouseMpmRenderScreenGridSpace;
	}

	if (m_zoomState && m_rightButtonDown) {
		m_zoomState = false;
		m_zoomFactor -= 0.5;
		m_zoomFactor = glm::max(1.0, m_zoomFactor);
		m_zoomPoint = m_mouseMpmRenderScreenGridSpace;
	}

	m_mpmGeometryEngine->HandleGeometryStates();
}

void mpm::MpmEngine::UpdatePointCloudData(std::string pointCloudStr)
{
	if (m_pointCloudMap.count(pointCloudStr)) {
		void* ptr = glMapNamedBuffer(m_pointCloudMap[pointCloudStr]->ssbo, GL_READ_ONLY);
		MaterialPoint* data = static_cast<MaterialPoint*>(ptr);
		size_t pCloudN = m_pointCloudMap[pointCloudStr]->N;
		m_pointCloudMap[pointCloudStr]->points = std::vector<MaterialPoint>(data, data + pCloudN);
		glUnmapNamedBuffer(m_pointCloudMap[pointCloudStr]->ssbo);
	}
}

void mpm::MpmEngine::UpdateNodeData()
{
	if (0 <= m_node[0] && m_node[0] < GRID_SIZE_X && 0 <= m_node[1] && m_node[1] < GRID_SIZE_Y) {
		void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);
		GridNode* data = static_cast<GridNode*>(ptr);
		m_gn = data[m_node[0] * GRID_SIZE_X + m_node[1]];
		glUnmapNamedBuffer(gridSSBO);
	}
}



void mpm::MpmEngine::MapCPUPointCloudsToGPU()
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		void* ptr = glMapNamedBuffer(pointCloudPair.second->ssbo, GL_WRITE_ONLY);
		MaterialPoint* data = static_cast<MaterialPoint*>(ptr);
		memcpy(data, pointCloudPair.second->points.data(), pointCloudPair.second->points.size() * sizeof(MaterialPoint));
		glUnmapNamedBuffer(pointCloudPair.second->ssbo);
	}
}

void mpm::MpmEngine::MapCPUPointCloudToGPU(std::shared_ptr<PointCloud> pointCloud)
{
	void* ptr = glMapNamedBuffer(pointCloud->ssbo, GL_WRITE_ONLY);
	MaterialPoint* data = static_cast<MaterialPoint*>(ptr);
	memcpy(data, pointCloud->points.data(), pointCloud->points.size() * sizeof(MaterialPoint));
	glUnmapNamedBuffer(pointCloud->ssbo);
}

void mpm::MpmEngine::MapCPUGridToGPU()
{
	void* ptr = glMapNamedBuffer(gridSSBO, GL_WRITE_ONLY);
	GridNode* data = static_cast<GridNode*>(ptr);
	memcpy(data, m_grid->nodes.data(), m_grid->nodes.size() * sizeof(GridNode));
	glUnmapNamedBuffer(gridSSBO);
}

void mpm::MpmEngine::MapGPUPointCloudsToCPU()
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		void* ptr = glMapNamedBuffer(pointCloudPair.second->ssbo, GL_WRITE_ONLY);
		MaterialPoint* data = static_cast<MaterialPoint*>(ptr);
		memcpy(pointCloudPair.second->points.data(), data, pointCloudPair.second->points.size() * sizeof(MaterialPoint));
		glUnmapNamedBuffer(pointCloudPair.second->ssbo);
	}
}

void mpm::MpmEngine::MapGPUPointCloudToCPU(std::shared_ptr<PointCloud> pointCloud)
{
	void* ptr = glMapNamedBuffer(pointCloud->ssbo, GL_WRITE_ONLY);
	MaterialPoint* data = static_cast<MaterialPoint*>(ptr);
	memcpy(pointCloud->points.data(), data, pointCloud->points.size() * sizeof(MaterialPoint));
	glUnmapNamedBuffer(pointCloud->ssbo);
}

void mpm::MpmEngine::MapGPUGridToCPU()
{
	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);
	GridNode* data = static_cast<GridNode*>(ptr);
	memcpy(m_grid->nodes.data(), data, m_grid->nodes.size() * sizeof(GridNode));
	glUnmapNamedBuffer(gridSSBO);
}
