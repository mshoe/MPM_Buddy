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

	// first compile shaders
	m_gReset = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\gResetNodes.comp"}, "shaders\\compute\\mpm_header.comp");
	m_p2gScatter = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\p2gScatterParticleAndUpdateNodes.comp"}, "shaders\\compute\\mpm_header.comp");
	//m_p2gGather = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\p2gGatherParticlesAndUpdateNode.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gUpdate = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\gUpdateNodes.comp"}, "shaders\\compute\\mpm_header.comp");

	m_g2pGather = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\g2pGatherNodesAndUpdateParticle.comp"}, "shaders\\compute\\mpm_header.comp");
	m_p2gCalcVolumes = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\p2gCalculateVolumes.comp"}, "shaders\\compute\\mpm_header.comp");
	m_g2pCalcVolumes = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\g2pCalculateVolumes.comp"}, "shaders\\compute\\mpm_header.comp");

	m_pSetDeformationGradients = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\interactive\\pSetDeformationGradients.comp"}, "shaders\\compute\\mpm_header.comp");
	m_pMultDeformationGradients = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\interactive\\pMultiplyDeformationGradients.comp"}, "shaders\\compute\\mpm_header.comp");

	// implict time integration shaders
	m_p2g2pDeltaForce = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\g2p2gDeltaForce.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsInitPart1 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_InitPart1.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsInitPart2 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_InitPart2.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsInitPart3 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_InitPart3.comp"}, "shaders\\compute\\mpm_header.comp");

	m_gConjugateResidualsStepPart1 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_StepPart1.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsStepPart2 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_StepPart2.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsConclusion = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_Conclusion.comp"}, "shaders\\compute\\mpm_header.comp");

	glCreateVertexArrays(1, &VisualizeVAO);

	// RENDERING SHADERS

	m_pPointCloudShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\pointCloud.vs"}, std::vector<std::string>{"shaders\\graphics\\pointCloudPassThrough.gs"}, std::vector<std::string>{"shaders\\graphics\\pointCloud.fs"}, "shaders\\compute\\mpm_header.comp");
	//m_pPointCloudVectorShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\pointCloud.vs"}, std::vector<std::string>{"shaders\\graphics\\pointCloudVector.gs"}, std::vector<std::string>{"shaders\\graphics\\pointCloud.fs"}, "shaders\\compute\\mpm_header.comp");
	//m_pPointCloudShader = std::make_unique<StandardShader>(std::vector<std::string>{"shaders\\graphics\\pointCloud.vs"}, std::vector<std::string>{}, std::vector<std::string>{"shaders\\graphics\\pointCloud.fs"}, "shaders\\compute\\mpm_header.comp");
	m_mouseShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\mouseShader.vs"}, std::vector<std::string>{}, std::vector<std::string>{"shaders\\graphics\\mouseShader.fs"}, "shaders\\compute\\mpm_header.comp");
	//m_nodeShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\nodeSelectionShader.vs"}, std::vector<std::string>{"shaders\\graphics\\nodeSelectionShader.fs"}, "shaders\\compute\\mpm_header.comp");

	m_zoomWindowShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\zoomWindow.vs"}, std::vector<std::string>{}, std::vector<std::string>{"shaders\\graphics\\zoomWindow.fs"}, "shaders\\compute\\mpm_header.comp");
	m_gridShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\gridShader.vs"}, std::vector<std::string>{"shaders\\graphics\\gridShaderPassThrough.gs"}, std::vector<std::string>{"shaders\\graphics\\gridShader.fs"}, "shaders\\compute\\mpm_header.comp");
	m_gridShaderVector = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\gridShader.vs"}, std::vector<std::string>{"shaders\\graphics\\gridShaderVector.gs"}, std::vector<std::string>{"shaders\\graphics\\gridShader.fs"}, "shaders\\compute\\mpm_header.comp");
	m_gridShaderMarchingSquares = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\marchingSquares.vs"}, std::vector<std::string>{"shaders\\graphics\\marchingSquares.gs"}, std::vector<std::string>{"shaders\\graphics\\marchingSquares.fs"}, "shaders\\compute\\mpm_header.comp");

	m_borderShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\polygon.vs"}, std::vector<std::string>{"shaders\\graphics\\border.gs"}, std::vector<std::string>{"shaders\\graphics\\polygon.fs"}, "shaders\\compute\\mpm_header.comp");
	m_polygonShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\polygon.vs"}, std::vector<std::string>{"shaders\\graphics\\polygon.gs"}, std::vector<std::string>{"shaders\\graphics\\polygon.fs"}, "shaders\\compute\\mpm_header.comp");
	m_pwLineShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\polygon.vs"}, std::vector<std::string>{"shaders\\graphics\\pwLine.gs"}, std::vector<std::string>{"shaders\\graphics\\polygon.fs"}, "shaders\\compute\\mpm_header.comp");


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
	m_neoHookeanParameters.youngMod = 90000.0;
	m_neoHookeanParameters.poisson = 0.3;
	m_neoHookeanParameters.particleSpacing = 0.25;
	m_neoHookeanParameters.density = 40;
	
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

	if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) {
		for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
			SetDeformationGradients(pointCloudPair.first, mat2(1.0), m_setFp);
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

		if (glfwGetKey(window, GLFW_KEY_0) == GLFW_PRESS) {
			MpmReset();
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
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	
	if (m_paused && m_rightButtonDown) {
		ClearCreateStates();
	}

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

	if (m_paused && m_createCircleState && m_leftButtonDown)
	{
		m_createCircleState = false;

		std::cout << "Mouse position is at (" << m_mpm_mouse.x << ", " << m_mpm_mouse.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_circleCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Circle shape(vec2(m_mpm_mouse.x * GRID_SIZE_X, m_mpm_mouse.y * GRID_SIZE_Y), m_circle_r);
		std::string circleID = "circle" + std::to_string(m_circleCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		/*color.x = (float)glm::clamp(m_color[0], 0, 255) / 255.f;
		color.y = (float)glm::clamp(m_color[1], 0, 255) / 255.f;
		color.z = (float)glm::clamp(m_color[2], 0, 255) / 255.f;*/

		real inner_rounding = m_circle_r - m_circle_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(circleID, shape, real(m_chunks_x) * real(CHUNK_WIDTH), real(m_chunks_y) * real(CHUNK_WIDTH), inner_rounding, m_circle_rounding, m_mpParameters, m_comodel, sdf::SDF_OPTION::HOLLOW, false, m_fixedPointCloud, m_initVelocity, color);

		t2 = high_resolution_clock::now();

		

		std::cout << "Finished generating " << pointCloud->N << " points for '" << circleID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_createRectState && m_leftButtonDown)
	{
		m_createRectState = false;

		std::cout << "Mouse position is at (" << m_mpm_mouse.x << ", " << m_mpm_mouse.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_rectCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Rectangle shape(vec2(m_mpm_mouse.x * GRID_SIZE_X, m_mpm_mouse.y * GRID_SIZE_Y), m_rect_b, m_rect_h);
		std::string rectID = "rect" + std::to_string(m_rectCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		

		real inner_rounding = glm::min(m_rect_b, m_rect_h) - m_rect_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(rectID, shape, real(m_chunks_x) * real(CHUNK_WIDTH), real(m_chunks_y) * real(CHUNK_WIDTH), inner_rounding, m_rect_rounding, m_mpParameters, m_comodel, sdf::SDF_OPTION::HOLLOW, false, m_fixedPointCloud, m_initVelocity, color);

		t2 = high_resolution_clock::now();



		std::cout << "Finished generating " << pointCloud->N << " points for '" << rectID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_createIsoTriState && m_leftButtonDown)
	{
		m_createIsoTriState = false;

		std::cout << "Mouse position is at (" << m_mpm_mouse.x << ", " << m_mpm_mouse.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_isoTriCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::IsoscelesTriangle shape(vec2(m_mpm_mouse.x * GRID_SIZE_X, m_mpm_mouse.y * GRID_SIZE_Y), m_iso_tri_b, m_iso_tri_h);
		std::string isoTriID = "isoTri" + std::to_string(m_isoTriCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

		real inner_rounding = glm::min(m_iso_tri_b, m_iso_tri_h) - m_iso_tri_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(isoTriID, shape, real(m_chunks_x)*real(CHUNK_WIDTH), real(m_chunks_y)*real(CHUNK_WIDTH), inner_rounding, m_iso_tri_rounding, m_mpParameters, m_comodel, sdf::SDF_OPTION::HOLLOW, false, m_fixedPointCloud, m_initVelocity, color);

		t2 = high_resolution_clock::now();


		std::cout << "Finished generating " << pointCloud->N << " points for '" << isoTriID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_selectNodeState && m_leftButtonDown) {
		m_selectNodeState = false;

		real mouseX = glm::clamp(m_mpm_mouse.x, 0.0, 1.0);
		real mouseY = glm::clamp(m_mpm_mouse.y, 0.0, 1.0);

		m_node[0] = glm::clamp((int)(mouseX * GRID_SIZE_X), 0, GRID_SIZE_X - 1);
		m_node[1] = glm::clamp((int)(mouseY * GRID_SIZE_Y), 0, GRID_SIZE_Y - 1);
	}

	if (m_paused && m_addPolygonVertexState && m_leftButtonDown) {
		m_addPolygonVertexState = false;

		real mouseX = m_mpm_mouse.x;
		real mouseY = m_mpm_mouse.y;

		real vertexX = mouseX * (real)GRID_SIZE_X;
		real vertexY = mouseY * (real)GRID_SIZE_Y;

		vec2 v = vec2(vertexX, vertexY);

		m_polygon->AddVertex(v);
	}

	if (m_paused && m_addPWLineVertexState && m_leftButtonDown) {
		m_addPWLineVertexState = false;

		real mouseX = m_mpm_mouse.x;
		real mouseY = m_mpm_mouse.y;

		real vertexX = mouseX * (real)GRID_SIZE_X;
		real vertexY = mouseY * (real)GRID_SIZE_Y;

		vec2 v = vec2(vertexX, vertexY);

		m_pwLine->AddVertex(v);
	}
}

std::shared_ptr<PointCloud> mpm::MpmEngine::GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
	const real gridDimX, const real gridDimY, 
	const real inner_rounding, const real outer_rounding,
	const MaterialParameters &parameters,
	const GLuint comodel, enum class sdf::SDF_OPTION sdfOption,
	bool inverted, bool fixed,
	vec2 initialVelocity, glm::highp_fvec4 color)
{
	std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();

	pointCloud->color = color;
	pointCloud->parameters = parameters;
	pointCloud->mew = parameters.youngMod / (2.f + 2.f* parameters.poisson);
	pointCloud->lam = parameters.youngMod * parameters.poisson / ((1.f + parameters.poisson) * (1.f - 2.f * parameters.poisson));

	std::cout << "mew: " << pointCloud->mew << ", lam: " << pointCloud->lam << std::endl;
	m_mew = pointCloud->mew;
	m_lam = pointCloud->lam;

	pointCloud->comodel = comodel;

	pointCloud->fixed = fixed;

	real mass = parameters.particleSpacing * parameters.particleSpacing * parameters.density;
	
	// gen points from sdf
	for (real x = 0.0; x < gridDimX; x += parameters.particleSpacing) {
		for (real y = 0.0; y < gridDimY; y += parameters.particleSpacing) {
			
			glm::vec2 p(x, y);
			
			real sd = 0.0;
			switch (sdfOption) {
			case sdf::SDF_OPTION::NORMAL:
				sd = shape.Sdf(p);
				break;
			case sdf::SDF_OPTION::ROUNDED:
				sd = shape.SdfRounded(p, outer_rounding);
				break;
			case sdf::SDF_OPTION::HOLLOW:
				sd = shape.SdfHollow(p, inner_rounding, outer_rounding);
				break;
			default:
				break;
			}

			if (inverted)
				sd *= -1.0;

			if (sd < 0.0) {
				MaterialPoint mp(p, initialVelocity, GLreal(mass));
				// calculate mp.vol in a compute shader (not here)
				
				
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

void mpm::MpmEngine::GenPointCloudPolygon()
{
	if (!m_paused)
		return;

	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();


	

	std::string polygonID = "polygon" + std::to_string(m_polygonCount);
	m_polygonCount++;

	glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

	std::shared_ptr<PointCloud> pointCloud = GenPointCloud(polygonID, *m_polygon, real(m_chunks_x) * real(CHUNK_WIDTH), real(m_chunks_y) * real(CHUNK_WIDTH), 0.0, 0.0, m_mpParameters, m_comodel, sdf::SDF_OPTION::NORMAL, m_invertedSdf, m_fixedPointCloud, m_initVelocity, color);

	t2 = high_resolution_clock::now();

	std::cout << "Finished generating " << pointCloud->N << " points for '" << polygonID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";

}

void mpm::MpmEngine::GenPointCloudPWLine()
{
	if (!m_paused)
		return;

	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();




	std::string pwLineID = "line" + std::to_string(m_pwLineCount);
	m_pwLineCount++;

	glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

	std::shared_ptr<PointCloud> pointCloud = GenPointCloud(pwLineID, *m_pwLine, real(m_chunks_x) * real(CHUNK_WIDTH), real(m_chunks_y) * real(CHUNK_WIDTH), 0.0, m_pwLineRounding, m_mpParameters, m_comodel, sdf::SDF_OPTION::ROUNDED, false, m_fixedPointCloud, m_initVelocity, color);

	t2 = high_resolution_clock::now();

	std::cout << "Finished generating " << pointCloud->N << " points for '" << pwLineID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
}

//void mpm::MpmEngine::GenPointCloudLineDivider()
//{
//	if (!m_paused)
//		return;
//
//	using namespace std::chrono;
//	time_point<high_resolution_clock> t1;
//	time_point<high_resolution_clock> t2;
//	t1 = high_resolution_clock::now();
//
//
//	m_isoTriCount++;
//	//sdf::sdFunc dCircle(sdf::DemoCircle);
//	sdf::LineDivider shape1(m_line_m, m_line_b);
//	sdf::LineDivider shape2(m_line2_m, m_line2_b);
//	sdf::Intersection andShape;
//	andShape.shapes.push_back(shape1);
//	andShape.shapes.push_back(shape2);
//	std::string lineDivID = "lineDiv" + std::to_string(m_lineDivCount);
//
//	glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);
//
//	std::shared_ptr<PointCloud> pointCloud = GenPointCloud(lineDivID, andShape, GRID_SIZE_X, GRID_SIZE_Y, 0.0, 0.0, m_mpParameters, m_comodel, false, m_invertedSdf, m_fixedPointCloud, m_initVelocity, color);
//
//	t2 = high_resolution_clock::now();
//
//	std::cout << "Finished generating " << pointCloud->N << " points for '" << lineDivID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
//
//}

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

void mpm::MpmEngine::SelectNodesInShape(sdf::Shape& shape, const int gridDimX, const int gridDimY, const real inner_rounding, const real outer_rounding, sdf::SDF_OPTION sdfOption, bool inverted)
{
//	std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();

	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	GridNode* data = static_cast<GridNode*>(ptr);
	
	
	int count = 0;
	// gen points from sdf
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			GridNode currNode = data[i * GRID_SIZE_Y + j];

			glm::vec2 p;
			p.x = real(i);
			p.y = real(j);// (real(i), real(j));

			real sd = 0.0;
			switch (sdfOption) {
			case sdf::SDF_OPTION::NORMAL:
				sd = shape.Sdf(p);
				break;
			case sdf::SDF_OPTION::ROUNDED:
				sd = shape.SdfRounded(p, outer_rounding);
				break;
			case sdf::SDF_OPTION::HOLLOW:
				sd = shape.SdfHollow(p, inner_rounding, outer_rounding);
				break;
			default:
				break;
			}

			if (inverted)
				sd *= -1.0;

			if (sd < 0.0) {
				currNode.selected = true;
				//currNode.force = vec2(100.0, 100.0);
				count++;
			}
			else {
				//currNode.selected = false;
			}

			data[i * GRID_SIZE_Y + j] = currNode;
		}
	}
	std::cout << "Selected " << count << " nodes.\n";
	glUnmapNamedBuffer(gridSSBO);

	/*ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	data = static_cast<GridNode*>(ptr);
	int count2 = 0;
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			GridNode currNode = data[i * GRID_SIZE_Y + j];

			if (currNode.selected)
				count2++;
		}
	}

	std::cout << "count2 = " << count2 << std::endl;
	glUnmapNamedBuffer(gridSSBO);*/
	
}

void mpm::MpmEngine::ClearNodesSelected(const int gridDimX, const int gridDimY)
{
	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	GridNode* data = static_cast<GridNode*>(ptr);


	int count = 0;
	// gen points from sdf
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			GridNode currNode = data[i * GRID_SIZE_Y + j];

			if (currNode.selected) {
				currNode.selected = false;
				count++;
				data[i * GRID_SIZE_Y + j] = currNode;
			}

		}
	}
	std::cout << "Cleared selection of " << count << " nodes.\n";
	glUnmapNamedBuffer(gridSSBO);
}

void mpm::MpmEngine::CalculateNodalAccelerations(const int gridDimX, const int gridDimY, real accStr)
{
	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	GridNode* data = static_cast<GridNode*>(ptr);

	std::vector<std::vector<vec2>> nodal_accs(gridDimX, std::vector<vec2>(gridDimY, vec2(0.0)));

	// temp buffer for node selected var
	int count = 0;
	std::vector<std::vector<bool>> nodes_selected(gridDimX, std::vector<bool>(gridDimY, false));
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			if (data[i * GRID_SIZE_Y + j].selected) {
				nodes_selected[i][j] = data[i * GRID_SIZE_Y + j].selected;
				count++;
			}
		}
	}

	if (count == 0) {
		glUnmapNamedBuffer(gridSSBO);
		std::cout << "Can't compute accelerations since no nodes selected.\n";
		return;
	}

	int countIter = 0;
	bool finished = false;
	// gen points from sdf
	while (!finished) {
		finished = true;
		for (int i = 0; i < gridDimX; i++) {
			for (int j = 0; j < gridDimY; j++) {

				GridNode currNode = data[i * GRID_SIZE_Y + j];

				if (currNode.selected) {
					continue; // don't calculate nodal acceleration for marked nodes
				}

				finished = false;

				int left = (i - 1) * GRID_SIZE_Y + j;
				int right = (i + 1) * GRID_SIZE_Y + j;
				int up = i * GRID_SIZE_Y + j + 1;
				int down = i * GRID_SIZE_Y + j - 1;


				// if left exists
				if (i > 0 && data[left].selected) {
					nodal_accs[i][j].x -= accStr;
					nodes_selected[i][j] = true;
				}
				// if right exists
				if (i < gridDimX - 1 && data[right].selected) {
					nodal_accs[i][j].x += accStr;
					nodes_selected[i][j] = true;
				}
				// if up exists
				if (j < gridDimY - 1 && data[up].selected) {
					nodal_accs[i][j].y += accStr;
					nodes_selected[i][j] = true;
				}
				// if down exists 
				if (j > 0 && data[down].selected) {
					nodal_accs[i][j].y -= accStr;
					nodes_selected[i][j] = true;
				}

				//currNode.nodalAcceleration = acc;
				//currNode.selected = true;
				

				data[i * GRID_SIZE_Y + j] = currNode;
			}
		}

		// Write buffer to original data
		for (int i = 0; i < gridDimX; i++) {
			for (int j = 0; j < gridDimY; j++) {
				GridNode currNode = data[i * GRID_SIZE_Y + j];
				currNode.selected = nodes_selected[i][j];
				currNode.nodalAcceleration = nodal_accs[i][j];
				data[i * GRID_SIZE_Y + j] = currNode;
			}
		}

		countIter++;
	}
	std::cout << "Computed nodal accelerations in " << countIter << " iterations.\n";
	glUnmapNamedBuffer(gridSSBO);
}

void mpm::MpmEngine::ClearNodalAcclerations(const int gridDimX, const int gridDimY)
{
	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	GridNode* data = static_cast<GridNode*>(ptr);


	int count = gridDimX * gridDimY;
	// gen points from sdf
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			GridNode currNode = data[i * GRID_SIZE_Y + j];


			currNode.nodalAcceleration = vec2(0.0);
			currNode.selected = false;

			data[i * GRID_SIZE_Y + j] = currNode;
		}
	}
	std::cout << "Cleared " << count << " nodal accelerations.\n";
	glUnmapNamedBuffer(gridSSBO);
}

void mpm::MpmEngine::ChangeMaterialParameters(GLuint comodel)
{
	// save the current material parameters
	switch (m_comodel) {
	case NEO_HOOKEAN_ELASTICITY:
		m_neoHookeanParameters = m_mpParameters;
		break;
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
	case NEO_HOOKEAN_ELASTICITY:
		m_mpParameters = m_neoHookeanParameters;
		break;
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
