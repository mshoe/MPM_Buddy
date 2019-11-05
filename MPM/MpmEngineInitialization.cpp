#include "MpmEngine.h"

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

	m_polygonEditorShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "polygon.vs"},
		std::vector<std::string>{graphicsPath + "polygonEditor.gs"},
		std::vector<std::string>{graphicsPath + "polygon.fs"},
		std::vector<std::string>{});


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

void mpm::MpmEngine::InitPolygonEditorScreen()
{
	m_polygonEditorScreen = std::make_shared<ImGuiScreen>(vec2(450.0, 450.0));
	m_polygonEditorScreen->center = vec2(225.0, 225.0);
	//m_zoomWindow->screen_dimensions = vec2(450.0, 450.0);
	m_polygonEditorScreen->sim_dimensions = vec2(450.0, 450.0);
}