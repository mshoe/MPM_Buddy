#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"



bool mpm::MpmEngine::InitComputeShaderPipeline()
{
	using namespace std::chrono;

	glEnable(GL_PROGRAM_POINT_SIZE);

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	int work_group_size;
	glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &work_group_size);
	std::cout << "max work group size: " << work_group_size << std::endl;

	std::string computePath = "shaders\\compute\\";
	std::string mpmHeadersPath = "shaders\\compute\\mpmHeaders\\";
	std::string graphicsPath = "shaders\\graphics\\";
	std::string interactivePath = "shaders\\compute\\interactive\\";
	std::string implicitPath = "shaders\\compute\\implicit\\";

	// first compile shaders
	m_gReset = std::make_unique<ComputeShader>(
		std::vector<std::string>{computePath + "gResetNodes.comp"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_p2gScatter = std::make_unique<ComputeShader>(
		std::vector<std::string>{computePath + "p2gScatterParticleAndUpdateNodes.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp", mpmHeadersPath + "shapeFunctions.comp"});
	m_gUpdate = std::make_unique<ComputeShader>(
		std::vector<std::string>{computePath + "gUpdateNodes.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});

	m_g2pGather = std::make_unique<ComputeShader>(
		std::vector<std::string>{computePath + "g2pGatherNodesAndUpdateParticle.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp", mpmHeadersPath + "shapeFunctions.comp", mpmHeadersPath + "energyFunctions.comp"});
	m_p2gCalcVolumes = std::make_unique<ComputeShader>(
		std::vector<std::string>{computePath + "p2gCalculateVolumes.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp", mpmHeadersPath + "shapeFunctions.comp"});
	m_g2pCalcVolumes = std::make_unique<ComputeShader>(
		std::vector<std::string>{computePath + "g2pCalculateVolumes.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp", mpmHeadersPath + "shapeFunctions.comp"});


	// interactive / control shaders
	m_pSetDeformationGradients = std::make_unique<ComputeShader>(
		std::vector<std::string>{interactivePath + "pSetDeformationGradients.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_pMultDeformationGradients = std::make_unique<ComputeShader>(
		std::vector<std::string>{interactivePath + "pMultiplyDeformationGradients.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_pLassoTool = std::make_unique<ComputeShader>(
		std::vector<std::string>{interactivePath + "pLassoTool.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_pClearPointSelection = std::make_unique<ComputeShader>(
		std::vector<std::string>{interactivePath + "pClearPointSelection.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});



	// implict time integration shaders
	m_p2g2pDeltaForce = std::make_unique<ComputeShader>(
		std::vector<std::string>{implicitPath + "g2p2gDeltaForce.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp", mpmHeadersPath + "shapeFunctions.comp", mpmHeadersPath + "energyFunctions.comp"});
	m_gConjugateResidualsInitPart1 = std::make_unique<ComputeShader>(
		std::vector<std::string>{implicitPath + "gCR_InitPart1.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_gConjugateResidualsInitPart2 = std::make_unique<ComputeShader>(
		std::vector<std::string>{implicitPath + "gCR_InitPart2.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_gConjugateResidualsInitPart3 = std::make_unique<ComputeShader>(
		std::vector<std::string>{implicitPath + "gCR_InitPart3.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});

	m_gConjugateResidualsStepPart1 = std::make_unique<ComputeShader>(
		std::vector<std::string>{implicitPath + "gCR_StepPart1.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_gConjugateResidualsStepPart2 = std::make_unique<ComputeShader>(
		std::vector<std::string>{implicitPath + "gCR_StepPart2.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_gConjugateResidualsConclusion = std::make_unique<ComputeShader>(
		std::vector<std::string>{implicitPath + "gCR_Conclusion.comp"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});

	glCreateVertexArrays(1, &VisualizeVAO);

	// RENDERING SHADERS

	m_pPointCloudShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "pointCloud.vs"}, 
		std::vector<std::string>{graphicsPath + "pointCloudPassThrough.gs"}, 
		std::vector<std::string>{graphicsPath + "pointCloud.fs"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_mouseShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "mouseShader.vs"}, 
		std::vector<std::string>{}, 
		std::vector<std::string>{graphicsPath + "mouseShader.fs"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	
	m_zoomWindowShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "zoomWindow.vs"}, 
		std::vector<std::string>{}, 
		std::vector<std::string>{graphicsPath + "zoomWindow.fs"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_gridShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "gridShader.vs"}, 
		std::vector<std::string>{graphicsPath + "gridShaderPassThrough.gs"}, 
		std::vector<std::string>{graphicsPath + "gridShader.fs"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_gridShaderVector = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "gridShader.vs"}, 
		std::vector<std::string>{graphicsPath + "gridShaderVector.gs"}, 
		std::vector<std::string>{graphicsPath + "gridShader.fs"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_gridShaderMarchingSquares = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "marchingSquares.vs"}, 
		std::vector<std::string>{graphicsPath + "marchingSquares.gs"}, 
		std::vector<std::string>{graphicsPath + "marchingSquares.fs"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});

	m_borderShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "polygon.vs"}, 
		std::vector<std::string>{graphicsPath + "border.gs"}, 
		std::vector<std::string>{graphicsPath + "polygon.fs"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_polygonShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "polygon.vs"}, 
		std::vector<std::string>{graphicsPath + "polygon.gs"}, 
		std::vector<std::string>{graphicsPath + "polygon.fs"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_pwLineShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "polygon.vs"}, 
		std::vector<std::string>{graphicsPath + "pwLine.gs"}, 
		std::vector<std::string>{graphicsPath + "polygon.fs"}, 
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});


	m_openGLScreen = std::make_shared<OpenGLScreen>();
	m_openGLScreen->center = vec2(1350.0, 450.0);
	m_openGLScreen->screen_dimensions = vec2((real)SRC_WIDTH, (real)SRC_HEIGHT);
	m_openGLScreen->sim_dimensions = vec2(900.0, 900.0);

	InitZoomWindow();

	// Initialize the grid SSBO on the GPU
	m_grid = Grid(GRID_SIZE_X, GRID_SIZE_Y);

	glCreateBuffers(1, &gridSSBO);

	glNamedBufferStorage(
		gridSSBO,
		sizeof(GridNode) * GRID_SIZE_X * GRID_SIZE_Y,
		&(m_grid.nodes[0].m),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // adding write bit for debug purposes
	);

	// initialize material parameters here for now
	m_energyModels[size_t(ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY)].youngMod = 90000.0;
	m_energyModels[size_t(ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY)].poisson = 0.3;
	m_energyModels[size_t(ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY)].density = 40;
	
	m_energyModels[size_t(ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY)].youngMod = 90000.0;
	m_energyModels[size_t(ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY)].poisson = 0.3;
	m_energyModels[size_t(ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY)].density = 40;

	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].youngMod = 140000.0;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].poisson = 0.2;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].density = 40;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].crit_c = 0.025;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].crit_s = 0.0075;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].hardening = 10.0;

	m_mpParameters = m_energyModels[size_t(ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY)];

	m_polygon = std::make_shared<sdf::Polygon>();
	m_pwLine = std::make_shared<sdf::PWLine>();
	return true;
}

bool mpm::MpmEngine::CleanupComputeShaderPipeline()
{
	m_pointCloudMap.clear();
	glDeleteBuffers(1, &gridSSBO);
	glDeleteVertexArrays(1, &VisualizeVAO);
	return false;
}

void mpm::MpmEngine::InitZoomWindow() {
	m_zoomWindow = std::make_shared<ImGuiScreen>(vec2(450.0, 450.0));
	m_zoomWindow->center = vec2(225.0, 225.0);
	//m_zoomWindow->screen_dimensions = vec2(450.0, 450.0);
	m_zoomWindow->sim_dimensions = vec2(450.0, 450.0);

	
}

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

	//std::cout << xpos << ", " << ypos << std::endl;

	// goal: Map mouse click to normalized grid space, which is (0.0, 1.0) when inside the MPM grid.
	// Grid space can be found from the OpenGLScreen's center and dimensions

	//real width = m_openGLScreen->dimensions.x;
	real mpm_xpos = (xpos - m_openGLScreen->center.x + m_openGLScreen->sim_dimensions.x / 2.0) / m_openGLScreen->sim_dimensions.x;
	real mpm_ypos = (ypos - m_openGLScreen->center.y + m_openGLScreen->sim_dimensions.y / 2.0) / m_openGLScreen->sim_dimensions.y;

	// normalize mouse coordinates
	xpos = xpos / (real)SRC_WIDTH;
	ypos = ypos / (real)SRC_HEIGHT;

	m_leftButtonDown = (bool)left_click;
	m_rightButtonDown = (bool)right_click;

	//std::cout << mpm_xpos << ", " << mpm_ypos << std::endl;

	// y value is given inverted
	m_mpm_mouse = vec4(mpm_xpos, 1.0 - mpm_ypos, left_click, right_click);
	m_mouse = vec4(xpos, 1.0 - ypos, left_click, right_click);
}

void mpm::MpmEngine::HandleStates()
{
	

	if (m_zoomState && m_leftButtonDown) {
		m_zoomState = false;
		m_zoomFactor += 0.5;
		m_zoomPoint = vec2(m_mpm_mouse.x * (real)GRID_SIZE_X, m_mpm_mouse.y * (real)GRID_SIZE_Y);
	}

	if (m_zoomState && m_midButtonDown) {
		m_zoomState = false;
		m_zoomPoint = vec2(m_mpm_mouse.x * (real)GRID_SIZE_X, m_mpm_mouse.y * (real)GRID_SIZE_Y);
	}

	if (m_zoomState && m_rightButtonDown) {
		m_zoomState = false;
		m_zoomFactor -= 0.5;
		m_zoomFactor = glm::max(1.0, m_zoomFactor);
		m_zoomPoint = vec2(m_mpm_mouse.x * (real)GRID_SIZE_X, m_mpm_mouse.y * (real)GRID_SIZE_Y);
	}

	HandleGeometryStates();
}

void mpm::MpmEngine::PrintGridData()
{
	void *ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);
	std::cout << "Reading as GridNode: \n";
	GridNode *data = static_cast<GridNode*>(ptr);
	for (int i = 0; i < GRID_SIZE_X*GRID_SIZE_Y; i++) {
		GridNode gn = data[i];
		if (gn.m != 0.0 || gn.v.x != 0.0 || gn.v.y != 0.0) {
			std::cout << "Grid node: [" << i / GRID_SIZE_X << ", " << i % GRID_SIZE_X << "]" << std::endl;
			std::cout << gn << std::endl;
		}
	}
	glUnmapNamedBuffer(gridSSBO);
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
