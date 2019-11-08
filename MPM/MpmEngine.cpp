#include "MpmEngine.h"

void mpm::MpmEngine::Update()
{
	if (!m_paused) {
		switch (m_algo_code) {
		case MPM_ALGORITHM_CODE::GLSL:
			if (!m_rt) {
				MpmTimeStep_GLSL(m_dt);
				
			}
			else {
				real curr_dt = 0.0;
				real rt_dt = 1.0 / 60.0;
				while (curr_dt < rt_dt) {
					MpmTimeStep_GLSL(m_dt);
					curr_dt += m_dt;
				}
			}
			break;
		case MPM_ALGORITHM_CODE::CPP:
			if (!m_rt) {
				MpmTimeStep_CPP(m_dt);
				MapCPUPointCloudsToGPU();
				MapCPUGridToGPU();
			}
			else {
				real curr_dt = 0.0;
				real rt_dt = 1.0 / 60.0;
				while (curr_dt < rt_dt) {
					MpmTimeStep_CPP(m_dt);
					curr_dt += m_dt;
				}
				MapCPUPointCloudsToGPU();
				MapCPUGridToGPU();
			}
			break;
		default:
			break;
		}

		
	}
}

void mpm::MpmEngine::ProcessKeyboardInput(GLFWwindow* window, real lag)
{
	// don't process keyboard input when writing into imgui textboxes
	if (ImGui::GetIO().WantTextInput) {
		return;
	}

	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
		m_paused = true;


	if (glfwGetKey(window, GLFW_KEY_Z) == GLFW_PRESS) {
		m_zoomState = true;
	}

	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
		if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
			m_paused = false;
		}
	}

	/*if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) {
		for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
			SetDeformationGradients(pointCloudPair.first, mat2(1.0), mat2(1.0), false);
		}
	}*/

	if (m_paused) {

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

		if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS) {
			MpmReset_GLSL();
		}

		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
			m_addPolygonVertexState = true;
		}

		if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) {
			m_addPWLineVertexState = true;
		}
	}

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

	if (m_rightButtonDown) {
		ClearCreateStates();
	}

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

	HandleGeometryStates();
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


void mpm::MpmEngine::ChangeEnergyModel(ENERGY_MODEL comodel)
{
	// save the current material parameters

	if (comodel >= ENERGY_MODEL::Count) {
		return; // not sure if this error check makes sense
	}

	m_energyModels[size_t(m_comodel)] = m_mpParameters;
	m_comodel = comodel;
	m_mpParameters = m_energyModels[size_t(comodel)];
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

void mpm::MpmEngine::MapCPUGridToGPU()
{
	void* ptr = glMapNamedBuffer(gridSSBO, GL_WRITE_ONLY);
	GridNode* data = static_cast<GridNode*>(ptr);
	memcpy(data, m_grid.nodes.data(), m_grid.nodes.size() * sizeof(GridNode));
	glUnmapNamedBuffer(gridSSBO);
}
