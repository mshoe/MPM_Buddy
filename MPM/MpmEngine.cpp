#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"

real BSpline(real x) {
	return (x < 0.5) ? glm::step(0.0, x)*(0.75 - x * x) :
		glm::step(x, 1.5)*0.5*(1.5 - abs(x))*(1.5 - abs(x));
}

real BSplineSlope(real x) {
	return (x < 0.5) ? glm::step(0.0, x)*(-2.0 * x) :
		glm::step(x, 1.5)*(1.5 - abs(x))*x / abs(x);
}



bool mpm::MpmEngine::InitComputeShaderPipeline()
{
	using namespace std::chrono;

	glEnable(GL_PROGRAM_POINT_SIZE);

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	int work_group_size;
	glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &work_group_size);
	std::cout << "max work group size: " << work_group_size << std::endl;

	// first compile shaders
	m_gReset = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\gResetNodes.comp"}, "shaders\\compute\\mpm_header.comp");
	m_p2gScatter = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\p2gScatterParticleAndUpdateNodes.comp"}, "shaders\\compute\\mpm_header.comp");
	//m_p2gGather = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\p2gGatherParticlesAndUpdateNode.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gUpdate = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\gUpdateNodes.comp"}, "shaders\\compute\\mpm_header.comp");

	m_g2pGather = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\g2pGatherNodesAndUpdateParticle.comp"}, "shaders\\compute\\mpm_header.comp");
	m_p2gCalcVolumes = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\p2gCalculateVolumes.comp"}, "shaders\\compute\\mpm_header.comp");
	m_g2pCalcVolumes = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\g2pCalculateVolumes.comp"}, "shaders\\compute\\mpm_header.comp");

	m_pSetReferenceConfig = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\pSetReferenceConfig.comp"}, "shaders\\compute\\mpm_header.comp");

	// implict time integration shaders
	m_p2g2pDeltaForce = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\g2p2gDeltaForce.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsInitPart1 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_InitPart1.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsInitPart2 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_InitPart2.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsInitPart3 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_InitPart3.comp"}, "shaders\\compute\\mpm_header.comp");

	m_gConjugateResidualsStepPart1 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_StepPart1.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsStepPart2 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_StepPart2.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsConclusion = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_Conclusion.comp"}, "shaders\\compute\\mpm_header.comp");

	glCreateVertexArrays(1, &VisualizeVAO);

	m_pPointCloudShader = std::make_unique<StandardShader>(std::vector<std::string>{"shaders\\graphics\\pointCloud.vs"}, std::vector<std::string>{"shaders\\graphics\\pointCloud.fs"}, "shaders\\compute\\mpm_header.comp");
	m_mouseShader = std::make_unique<StandardShader>(std::vector<std::string>{"shaders\\graphics\\mouseShader.vs"}, std::vector<std::string>{"shaders\\graphics\\mouseShader.fs"}, "shaders\\compute\\mpm_header.comp");
	//m_nodeShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\nodeSelectionShader.vs"}, std::vector<std::string>{"shaders\\graphics\\nodeSelectionShader.fs"}, "shaders\\compute\\mpm_header.comp");

	m_openGLScreen = std::make_unique<OpenGLScreen>();

	// Initialize the grid SSBO on the GPU
	m_grid = Grid(GRID_SIZE_X, GRID_SIZE_Y);

	glCreateBuffers(1, &gridSSBO);

	glNamedBufferStorage(
		gridSSBO,
		sizeof(GridNode) * GRID_SIZE_X * GRID_SIZE_Y,
		&(m_grid.nodes[0].m),
		GL_MAP_READ_BIT
	);

	// initialize material parameters here for now
	m_fixedCorotatedParameters.youngMod = 90000.0;
	m_fixedCorotatedParameters.poisson = 0.3;
	m_fixedCorotatedParameters.particleSpacing = 0.25;
	m_fixedCorotatedParameters.density = 40;

	m_simpleSnowParameters.youngMod = 140000.0;
	m_simpleSnowParameters.poisson = 0.2;
	m_simpleSnowParameters.particleSpacing = 0.25;
	m_simpleSnowParameters.density = 40;
	m_simpleSnowParameters.crit_c = 0.025;
	m_simpleSnowParameters.crit_s = 0.0075;
	m_simpleSnowParameters.hardening = 10.0;

	m_mpParameters = m_fixedCorotatedParameters;
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

}

void mpm::MpmEngine::Render()
{
	if (!m_paused) {
		if (!m_rt) {
			MpmTimeStep(m_dt);
		}
		else {
			real curr_dt = 0.0;
			real rt_dt = 1.0 / 60.0;
			while (curr_dt < rt_dt) {
				MpmTimeStep(m_dt);
				curr_dt += m_dt;
			}
		}
	}
	// RENDER MOUSE SHADER
	m_mouseShader->Use();
	m_mouseShader->SetVec("iMouse", m_mouse);
	m_mouseShader->SetVec("iResolution", vec2(SRC_WIDTH, SRC_HEIGHT));
	m_mouseShader->SetBool("nodeGraphicsActive", m_nodeGraphicsActive);
	m_mouseShader->SetInt("nodeI", m_node[0]);
	m_mouseShader->SetInt("nodeJ", m_node[1]);
	m_openGLScreen->Render();

	// RENDER POINT CLOUD
	m_pPointCloudShader->Use();
	m_pPointCloudShader->SetReal("maxEnergyClamp", m_maxEnergyClamp);
	m_pPointCloudShader->SetReal("minEnergyClamp", m_minEnergyClamp);
	m_pPointCloudShader->SetBool("visualizeEnergy", m_visualizeEnergy);
	glBindVertexArray(VisualizeVAO);
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		m_pPointCloudShader->SetVec("pointCloudColor", pointCloudPair.second->color);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glDrawArrays(GL_POINTS, 0, (GLsizei)pointCloudPair.second->N);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
	glBindVertexArray(0);
}

void mpm::MpmEngine::ProcessKeyboardInput(GLFWwindow* window, real lag)
{
	// don't process keyboard input when writing into imgui textboxes
	if (ImGui::GetIO().WantTextInput) {
		return;
	}

	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
		m_paused = true;

	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
		if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
			m_paused = false;
		}
	}

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
	}

	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS) {
		m_selectNodeState = true;
	}
}

void mpm::MpmEngine::ProcessMouseInput(GLFWwindow* window, real lag)
{
	real xpos, ypos, left_click, right_click;
	glfwGetCursorPos(window, &xpos, &ypos);
	left_click = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) ? 1.0 : 0.0;
	right_click = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) ? 1.0 : 0.0;

	// since mpm engine occurs on right half of screen, xpos needs to be mapped there
	real mpm_xpos = (xpos - (real)SRC_WIDTH / 2.0) / (real)SRC_WIDTH * 2.0;

	// normalize mouse coordinates
	xpos = xpos / (real)SRC_WIDTH;
	ypos = ypos / (real)SRC_HEIGHT;

	m_leftButtonDown = (bool)left_click;
	m_rightButtonDown = (bool)right_click;
	// y value is given inverted
	m_mouse = vec4(mpm_xpos, 1.0 - ypos, left_click, right_click);
}

void mpm::MpmEngine::HandleStates()
{
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	
	if (m_paused && m_rightButtonDown) {
		ClearCreateStates();
	}

	if (m_paused && m_createCircleState && m_leftButtonDown)
	{
		m_createCircleState = false;

		std::cout << "Mouse position is at (" << m_mouse.x << ", " << m_mouse.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_circleCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Circle shape(vec2(m_mouse.x * GRID_SIZE_X, m_mouse.y * GRID_SIZE_Y), m_circle_r);
		std::string circleID = "circle" + std::to_string(m_circleCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		/*color.x = (float)glm::clamp(m_color[0], 0, 255) / 255.f;
		color.y = (float)glm::clamp(m_color[1], 0, 255) / 255.f;
		color.z = (float)glm::clamp(m_color[2], 0, 255) / 255.f;*/

		real inner_rounding = m_circle_r - m_circle_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(circleID, shape, GRID_SIZE_X, GRID_SIZE_Y, inner_rounding, m_circle_rounding, m_mpParameters, m_comodel, vec2(0.0, 0.0), color);

		t2 = high_resolution_clock::now();

		

		std::cout << "Finished generating " << pointCloud->N << " points for '" << circleID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_createRectState && m_leftButtonDown)
	{
		m_createRectState = false;

		std::cout << "Mouse position is at (" << m_mouse.x << ", " << m_mouse.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_rectCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Rectangle shape(vec2(m_mouse.x * GRID_SIZE_X, m_mouse.y * GRID_SIZE_Y), m_rect_b, m_rect_h);
		std::string rectID = "rect" + std::to_string(m_rectCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		

		real inner_rounding = glm::min(m_rect_b, m_rect_h) - m_rect_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(rectID, shape, GRID_SIZE_X, GRID_SIZE_Y, inner_rounding, m_rect_rounding, m_mpParameters, m_comodel, vec2(0.0, 0.0), color);

		t2 = high_resolution_clock::now();



		std::cout << "Finished generating " << pointCloud->N << " points for '" << rectID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_createIsoTriState && m_leftButtonDown)
	{
		m_createIsoTriState = false;

		std::cout << "Mouse position is at (" << m_mouse.x << ", " << m_mouse.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_isoTriCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::IsoscelesTriangle shape(vec2(m_mouse.x * GRID_SIZE_X, m_mouse.y * GRID_SIZE_Y), m_iso_tri_b, m_iso_tri_h);
		std::string isoTriID = "isoTri" + std::to_string(m_isoTriCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

		real inner_rounding = glm::min(m_iso_tri_b, m_iso_tri_h) - m_iso_tri_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(isoTriID, shape, GRID_SIZE_X, GRID_SIZE_Y, inner_rounding, m_iso_tri_rounding, m_mpParameters, m_comodel, glm::vec2(0.f, 0.f), color);

		t2 = high_resolution_clock::now();


		std::cout << "Finished generating " << pointCloud->N << " points for '" << isoTriID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_selectNodeState && m_leftButtonDown) {
		m_selectNodeState = false;

		real mouseX = glm::clamp(m_mouse.x, 0.0, 1.0);
		real mouseY = glm::clamp(m_mouse.y, 0.0, 1.0);

		m_node[0] = glm::clamp((int)(mouseX * GRID_SIZE_X), 0, GRID_SIZE_X - 1);
		m_node[1] = glm::clamp((int)(mouseY * GRID_SIZE_Y), 0, GRID_SIZE_Y - 1);
	}
}

std::shared_ptr<PointCloud> mpm::MpmEngine::GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
	const real gridDimX, const real gridDimY, 
	const real inner_rounding, const real outer_rounding,
	const MaterialParameters &parameters,
	const GLuint comodel,
	vec2 initialVelocity, glm::highp_fvec4 color)
{
	std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();

	pointCloud->color = color;
	pointCloud->parameters = parameters;
	pointCloud->mew = parameters.youngMod / (2.f + 2.f* parameters.poisson);
	pointCloud->lam = parameters.youngMod * parameters.poisson / ((1.f + parameters.poisson) * (1.f - 2.f * parameters.poisson));

	pointCloud->comodel = comodel;

	real mass = parameters.particleSpacing * parameters.particleSpacing * parameters.density;
	
	// gen points from sdf
	for (real x = 0.f; x < gridDimX; x += parameters.particleSpacing) {
		for (real y = 0.f; y < gridDimY; y += parameters.particleSpacing) {
			
			glm::vec2 p(x, y);
			real sd = shape.SdfHollow(p, inner_rounding, outer_rounding);
			if (sd < 0.f) {
				MaterialPoint mp;
				mp.x = p;
				mp.v = initialVelocity;
				mp.m = mass;
				// calculate mp.vol in a compute shader (not here)
				mp.B = mat2(0.0);
				mp.Fe = mat2(1.0);
				mp.Fp = mat2(1.0);
				mp.P = mat2(0.0); // initial Piola stress tensor is 0
				mp.FePolar_R = mat2(1.0);
				mp.FePolar_S = mat2(1.0);
				mp.FeSVD_U = mat2(1.0);
				mp.FeSVD_S = mat2(1.0);
				mp.FeSVD_V = mat2(1.0);
				
				pointCloud->points.push_back(mp);
			}
		}
	}
	pointCloud->N = pointCloud->points.size();

	if (pointCloud->N > 0) {

		// Create the SSBO for the point cloud, so it is stored on the GPU
		GLuint pointCloudSSBO;
		glCreateBuffers(1, &pointCloudSSBO);
		pointCloud->ssbo = pointCloudSSBO;
		glNamedBufferStorage(
			pointCloud->ssbo,
			sizeof(MaterialPoint) * pointCloud->points.size(),
			&(pointCloud->points.front().x.x),
			GL_MAP_READ_BIT
		);

		// Calculate volumes for the point cloud (volumes stored in SSBO on GPU)
		CalculatePointCloudVolumes(pointCloudID, pointCloud);


		m_pointCloudMap[pointCloudID] = pointCloud;
	}

	return pointCloud;
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

void mpm::MpmEngine::UpdateNodeData()
{
	if (0 <= m_node[0] && m_node[0] < GRID_SIZE_X && 0 <= m_node[1] && m_node[1] < GRID_SIZE_Y) {
		void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);
		GridNode* data = static_cast<GridNode*>(ptr);
		m_gn = data[m_node[0] * GRID_SIZE_X + m_node[1]];
		glUnmapNamedBuffer(gridSSBO);
	}
}

void mpm::MpmEngine::ChangeMaterialParameters(GLuint comodel)
{
	// save the current material parameters
	switch (m_comodel) {
	case FIXED_COROTATIONAL_ELASTICITY:
		m_fixedCorotatedParameters = m_mpParameters;
		break;
	case SIMPLE_SNOW:
		m_simpleSnowParameters = m_mpParameters;
		break;
	default:
		break;
	}
	m_comodel = comodel;
	switch (m_comodel) {
	case FIXED_COROTATIONAL_ELASTICITY:
		m_mpParameters = m_fixedCorotatedParameters;
		break;
	case SIMPLE_SNOW:
		m_mpParameters = m_simpleSnowParameters;
		break;
	default:
		break;
	}
}
