#include "MpmEngine.h"

mpm::MpmEngine::MpmEngine()
{
	{
		InitComputeShaderPipeline();
		InitEngines();

		// Create OpenGL render window based on grid dimensions

		

		m_openGLScreen = std::make_shared<OpenGLScreen>();
		m_openGLScreen->center = vec2(1350.0, 450.0);
		m_openGLScreen->screen_dimensions = vec2((real)SRC_WIDTH, (real)SRC_HEIGHT);
		m_openGLScreen->sim_dimensions = vec2(900.0, 900.0);
		

		InitMpmSpace(64, 64);
	}
}

bool mpm::MpmEngine::InitEngines()
{
	m_mpmGeometryEngine = std::make_shared<MpmGeometryEngine>();
	m_mpmControlEngine = std::make_shared<MpmControlEngine>();
	m_mpmAlgorithmEngine = std::make_shared<MpmAlgorithmEngine>();

	m_mpmGeometryEngine->SetMpmEngine(this);
	m_mpmGeometryEngine->SetAlgorithmEngine(m_mpmAlgorithmEngine);
	m_mpmControlEngine->SetMpmEngine(this);
	m_mpmControlEngine->SetAlgorithmEngine(m_mpmAlgorithmEngine);
	m_mpmControlEngine->SetMpmGeometryEngine(m_mpmGeometryEngine);
	m_mpmAlgorithmEngine->SetMpmEngine(this);
	m_mpmAlgorithmEngine->SetMpmControlEngine(m_mpmControlEngine);
	m_mpmAlgorithmEngine->SetMpmGeometryEngine(m_mpmGeometryEngine);

	
	return true;
}

bool mpm::MpmEngine::InitMpmSpace(int grid_dim_x, int grid_dim_y)
{
	InitGrid(grid_dim_x, grid_dim_y);
	InitZoomWindow();
	InitMpmRenderWindow();
	return true;
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

	using namespace ShaderPaths;

	

	glCreateVertexArrays(1, &VisualizeVAO);

	// RENDERING SHADERS

	/*m_pPointCloudShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsMPPath + "pointCloud.vs"},
		std::vector<std::string>{graphicsMPPath + "pointCloudPassThrough.gs"},
		std::vector<std::string>{graphicsMPPath + "pointCloud.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});*/

	m_pPointCloudShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsMPPath + "MaterialPoints.vs"},
		std::vector<std::string>{graphicsMPPath + "MaterialPoints.gs"},
		std::vector<std::string>{graphicsMPPath + "MaterialPoints.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});


	m_mouseShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "mouseShader.vs"},
		std::vector<std::string>{},
		std::vector<std::string>{graphicsPath + "mouseShader.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});

	m_zoomWindowShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsPath + "zoomWindow.vs"},
		std::vector<std::string>{},
		std::vector<std::string>{graphicsPath + "zoomWindow.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});
	m_gridShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsGridPath + "gridShader.vs"},
		std::vector<std::string>{graphicsGridPath + "gridShaderPassThrough.gs"},
		std::vector<std::string>{graphicsGridPath + "gridShader.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});
	m_gridShaderVector = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsGridPath + "gridShader.vs"},
		std::vector<std::string>{graphicsGridPath + "gridShaderVector.gs"},
		std::vector<std::string>{graphicsGridPath + "gridShader.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});
	/*m_gridShaderMarchingSquares = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsGridPath + "marchingSquares.vs"},
		std::vector<std::string>{graphicsGridPath + "marchingSquares.gs"},
		std::vector<std::string>{graphicsGridPath + "marchingSquares.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});
	m_gridDensityShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsGridPath + "densityField.vs"},
		std::vector<std::string>{},
		std::vector<std::string>{graphicsGridPath + "densityField.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp", mpmHeadersPath + "shapeFunctions.comp"});*/

	m_borderShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsGeometryPath + "polygon.vs"},
		std::vector<std::string>{graphicsGridPath + "border.gs"},
		std::vector<std::string>{graphicsGeometryPath + "polygon.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});

	


	

	

	

	
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

void mpm::MpmEngine::InitMpmRenderWindow()
{
	real max_dim = glm::max(m_grid->grid_dim_x, m_grid->grid_dim_y);
	vec2 max_dims = vec2(max_dim, max_dim);
	vec2 norm_dims = vec2(m_grid->grid_dim_x, m_grid->grid_dim_y) / max_dims;
	vec2 sim_dimensions = vec2(800, 800) * norm_dims;

	m_mpmRenderWindow = std::make_shared<ImGuiScreen>(sim_dimensions);
	m_mpmRenderWindow->center = vec2(400.0, 400.0);
	//m_zoomWindow->screen_dimensions = vec2(450.0, 450.0);
	//m_mpmRenderWindow->sim_dimensions = vec2(800.0, 800.0);

	
	m_mpmRenderWindow->sim_dimensions = sim_dimensions;
}

//void mpm::MpmEngine::InitPolygonEditorScreen()
//{
//	m_polygonEditorScreen = std::make_shared<ImGuiScreen>(vec2(450.0, 450.0));
//	m_polygonEditorScreen->center = vec2(225.0, 225.0);
//	//m_zoomWindow->screen_dimensions = vec2(450.0, 450.0);
//	m_polygonEditorScreen->sim_dimensions = vec2(450.0, 450.0);
//}