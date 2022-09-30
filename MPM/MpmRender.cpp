#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"




void mpm::MpmEngine::MpmRender()
{

	m_mpmRenderWindow->BindFBO();
	glViewport(0, 0, (GLsizei)m_mpmRenderWindow->screen_dimensions.x, (GLsizei)m_mpmRenderWindow->screen_dimensions.y);
	glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
	glClear(GL_COLOR_BUFFER_BIT);
	RenderScreenShader(m_zoomPoint, 1.0, m_mpmRenderWindow);

	if (m_viewGridDensity) {
		RenderDensityField(m_zoomPoint, 1.0, 2, gridSSBO, false, m_mpmRenderWindow, m_gridDensityShader);
	}

	if (m_viewPointClouds) {
		RenderPointClouds(m_zoomPoint, 1.0, m_mpmRenderWindow, m_pPointCloudShader);
	}
	if (m_viewGrid) {
		RenderGrid(m_zoomPoint, 1.0, m_mpmRenderWindow, m_gridShader);
		if (m_viewGridVector) {
			RenderGrid(m_zoomPoint, 1.0, m_mpmRenderWindow, m_gridShaderVector);
		}
	}
	/*if (m_viewMarchingSquares) {
		RenderMarchingSquares(m_zoomPoint, 1.0, m_mpmRenderWindow, m_gridShaderMarchingSquares);
	}*/
	// Render Zoom Border
	if (m_showZoomBorder) {
		m_zoomWindowShader->Use();
		m_zoomWindowShader->SetReal("zoomFactor", m_zoomFactor);
		m_zoomWindowShader->SetVec("zoomPoint", m_zoomPoint);
		m_zoomWindow->RenderLineLoop();
	}

	m_mpmGeometryEngine->Render(m_zoomPoint, 1.0, m_mpmRenderWindow);
	//m_mpmControlEngine->Render(m_zoomPoint, 1.0, m_mpmRenderWindow);

	



	RenderGridBorder(m_zoomPoint, 1.0, m_mpmRenderWindow, m_borderShader);

	m_mpmRenderWindow->UnbindFBO();

}

void mpm::MpmEngine::ZoomRender()
{
	m_zoomWindow->BindFBO();
	glViewport(0, 0, (GLsizei)m_zoomWindow->screen_dimensions.x, (GLsizei)m_zoomWindow->screen_dimensions.y);
	glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
	glClear(GL_COLOR_BUFFER_BIT);
	//RenderScreenShader(m_zoomPoint, m_zoomFactor, m_zoomWindow);

	if (m_viewGridDensity) {
		RenderDensityField(m_zoomPoint, m_zoomFactor, 2, gridSSBO, false, m_zoomWindow, m_gridDensityShader);
	}

	if (m_viewPointClouds) {
		RenderPointClouds(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_pPointCloudShader);
	}
	if (m_viewGrid) {
		RenderGrid(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_gridShader);
		if (m_viewGridVector) {
			RenderGrid(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_gridShaderVector);
		}
	}
	// Render polygon

	//m_mpmGeometryEngine->Render(m_zoomPoint, m_zoomFactor, m_zoomWindow);
	//m_mpmControlEngine->Render(m_zoomPoint, m_zoomFactor, m_zoomWindow);

	RenderGridBorder(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_borderShader);

	m_zoomWindow->UnbindFBO();
}

void mpm::MpmEngine::Render()
{
	glViewport(0, 0, (GLsizei)m_openGLScreen->screen_dimensions.x, (GLsizei)m_openGLScreen->screen_dimensions.y);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	if (m_imguiMpmRenderWindow)
		MpmRender();
	if (m_imguiZoomWindow)
		ZoomRender();
}

void mpm::MpmEngine::RenderScreenShader(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen)
{
	m_mouseShader->Use();
	m_mouseShader->SetVec("mouse", m_mouseMpmRenderScreenGridSpaceFull);
	//m_mouseShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	//m_mouseShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	m_mouseShader->SetVec("screenResolution", openGLScreen->screen_dimensions);
	m_mouseShader->SetBool("nodeGraphicsActive", m_nodeGraphicsActive);
	m_mouseShader->SetInt("selectedNodeI", m_node[0]);
	m_mouseShader->SetInt("selectedNodeJ", m_node[1]);
	m_mouseShader->SetReal("zoomFactor", zoomFactor);
	m_mouseShader->SetVec("zoomPoint", zoomPoint);
	m_mouseShader->SetInt("GRID_SIZE_X", (int)m_grid->grid_dim_x);
	m_mouseShader->SetInt("GRID_SIZE_Y", (int)m_grid->grid_dim_y);

	glm::highp_fvec4 backgroundColor = glm::highp_fvec4(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
	m_mouseShader->SetVec("backgroundColor", backgroundColor);
	openGLScreen->Render();
}

void mpm::MpmEngine::RenderPointCloud(std::shared_ptr<PointCloud> pointCloud, vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pointShader)
{
	// RENDER POINT CLOUD
	pointShader->Use();
	pointShader->SetReal("maxSpeedClamp", m_maxSpeedClamp);
	pointShader->SetReal("minSpeedClamp", m_minSpeedClamp);
	pointShader->SetBool("visualizeSpeed", m_visualizeSpeed);
	pointShader->SetReal("maxEnergyClamp", m_maxEnergyClamp);
	pointShader->SetReal("minEnergyClamp", m_minEnergyClamp);
	pointShader->SetBool("visualizeEnergy", m_visualizeEnergy);
	m_mpmGeometryEngine->SetSelectionUniforms(pointShader);
	pointShader->SetReal("zoomFactor", zoomFactor);
	pointShader->SetVec("zoomPoint", zoomPoint);
	pointShader->SetInt("GRID_SIZE_X", (int)m_grid->grid_dim_x);
	pointShader->SetInt("GRID_SIZE_Y", (int)m_grid->grid_dim_y);
	// iResolution and iSourceResolution should be same for the zoom window we make, and iCenter should be the actual center
	//m_pPointCloudShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	//m_pPointCloudShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	//m_pPointCloudShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	//m_pPointCloudShader->SetVec("iCenter", m_openGLScreen->center);
	glBindVertexArray(VisualizeVAO);
	pointShader->SetVec("pointCloudColor", pointCloud->color);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
	glDrawArrays(GL_POINTS, 0, (GLsizei)pointCloud->N);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderPointClouds(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pointShader)
{

	// RENDER POINT CLOUD
	pointShader->Use();
	pointShader->SetReal("maxSpeedClamp", m_maxSpeedClamp);
	pointShader->SetReal("minSpeedClamp", m_minSpeedClamp);
	pointShader->SetBool("visualizeSpeed", m_visualizeSpeed);
	pointShader->SetReal("maxEnergyClamp", m_maxEnergyClamp);
	pointShader->SetReal("minEnergyClamp", m_minEnergyClamp);
	pointShader->SetBool("visualizeEnergy", m_visualizeEnergy);
	m_mpmGeometryEngine->SetSelectionUniforms(pointShader);
	pointShader->SetReal("zoomFactor", zoomFactor);
	pointShader->SetVec("zoomPoint", zoomPoint);
	pointShader->SetInt("GRID_SIZE_X", (int)m_grid->grid_dim_x);
	pointShader->SetInt("GRID_SIZE_Y", (int)m_grid->grid_dim_y);
	// iResolution and iSourceResolution should be same for the zoom window we make, and iCenter should be the actual center
	//m_pPointCloudShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	//m_pPointCloudShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	//m_pPointCloudShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	//m_pPointCloudShader->SetVec("iCenter", m_openGLScreen->center);
	glBindVertexArray(VisualizeVAO);
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		pointShader->SetVec("pointCloudColor", pointCloudPair.second->color);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glDrawArrays(GL_POINTS, 0, (GLsizei)pointCloudPair.second->N);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderGrid(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader)
{
	gridShader->Use();
	gridShader->SetReal("zoomFactor", zoomFactor);
	gridShader->SetVec("zoomPoint", zoomPoint);
	// iResolution and iSourceResolution should be same for the zoom window we make, and iCenter should be the actual center
	//gridShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	//gridShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	//gridShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	gridShader->SetBool("viewGridMass", m_viewGridMass);
	gridShader->SetReal("maxNodeMassClamp", m_maxNodeMassClamp);
	gridShader->SetReal("minNodeMassClamp", m_minNodeMassClamp);
	gridShader->SetReal("maxPointSize", m_maxNodeMassPointSize);
	gridShader->SetReal("minNodeMassPointSize", m_minNodeMassPointSize);
	gridShader->SetReal("maxNodeMassPointSize", m_maxNodeMassPointSize);
	gridShader->SetInt("gridPointSizeScalingOption", m_gridPointSizeScalingOption);
	gridShader->SetBool("nodeGraphicsActive", m_nodeGraphicsActive);
	gridShader->SetInt("selectedNodeI", m_node[0]);
	gridShader->SetInt("selectedNodeJ", m_node[1]);
	gridShader->SetReal("dt", m_mpmAlgorithmEngine->m_dt);
	gridShader->SetuInt("selectedVector", unsigned int(m_gridVectorOption));
	gridShader->SetReal("maxGridVectorLength", m_maxGridVectorLength);
	gridShader->SetReal("maxGridVectorVisualLength", m_maxGridVectorVisualLength);

	gridShader->SetInt("GRID_SIZE_X", m_grid->grid_dim_x);
	gridShader->SetInt("GRID_SIZE_Y", m_grid->grid_dim_y);
	gridShader->SetInt("SCREEN_DIM_X", (int)m_mpmRenderWindow->sim_dimensions.x);
	gridShader->SetInt("SCREEN_DIM_Y", (int)m_mpmRenderWindow->sim_dimensions.y);
	//gridShader->SetBool("collectiveNodeGraphics", m_collectiveNodeSelectionGraphics);

	glBindVertexArray(VisualizeVAO);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)(m_grid->grid_dim_x * m_grid->grid_dim_y));
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderMarchingSquares(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader)
{
	gridShader->Use();
	gridShader->SetReal("zoomFactor", zoomFactor);
	gridShader->SetVec("zoomPoint", zoomPoint);
	gridShader->SetReal("isoMass", m_isoMass);
	gridShader->SetVec("mscolor", m_marchingSquaresColor);
	gridShader->SetReal("dt", m_mpmAlgorithmEngine->m_dt);

	glBindVertexArray(VisualizeVAO);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)(m_grid->grid_dim_x * m_grid->grid_dim_y));
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderGridBorder(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> borderShader)
{
	// Render the grid border
	borderShader->Use();
	borderShader->SetInt("CHUNKS_X", m_chunks_x);
	borderShader->SetInt("CHUNKS_Y", m_chunks_y);
	borderShader->SetVec("zoomPoint", zoomPoint);
	borderShader->SetReal("zoomFactor", zoomFactor);
	if (m_mpmAlgorithmEngine->m_algo_code == MpmAlgorithmEngine::MPM_ALGORITHM_CODE::CPP) {
		borderShader->SetInt("chunk_size_x", m_mpmAlgorithmEngine->m_cppChunkX);
		borderShader->SetInt("chunk_size_y", m_mpmAlgorithmEngine->m_cppChunkY);
	}
	else {
		borderShader->SetInt("chunk_size_x", 32);
		borderShader->SetInt("chunk_size_y", 32);
	}
	glBindVertexArray(VisualizeVAO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)1);
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderDensityField(vec2 zoomPoint, real zoomFactor, int binding, GLuint ssbo, bool controlMode, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> densityShader)
{
	densityShader->Use();
	//m_mouseShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	//m_mouseShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	densityShader->SetReal("zoomFactor", zoomFactor);
	densityShader->SetVec("zoomPoint", zoomPoint);
	densityShader->SetVec("screenResolution", openGLScreen->sim_dimensions);
	//std::cout << openGLScreen->screen_dimensions.x << ", " << m_openGLScreen->screen_dimensions.y << std::endl;
	glm::highp_fvec4 backgroundColor = glm::highp_fvec4(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
	densityShader->SetVec("backgroundColor", backgroundColor);
	glm::highp_fvec4 densityColor = glm::highp_fvec4(m_densityColor[0], m_densityColor[1], m_densityColor[2], m_densityColor[3]);
	densityShader->SetVec("maxDensityColor", densityColor);
	glm::highp_fvec4 mediumDensityColor = glm::highp_fvec4(m_mediumDensityColor[0], m_mediumDensityColor[1], m_mediumDensityColor[2], m_mediumDensityColor[3]);
	densityShader->SetVec("mediumDensityColor", mediumDensityColor);
	glm::highp_fvec4 minDensityColor = glm::highp_fvec4(m_minDensityColor[0], m_minDensityColor[1], m_minDensityColor[2], m_minDensityColor[3]);
	densityShader->SetVec("minDensityColor", minDensityColor);
	densityShader->SetBool("useColorSpectrum", m_useColorSpectrum);
	densityShader->SetReal("maxMass", m_gridMaxMass);
	densityShader->SetReal("mediumMass", m_gridMediumMass);
	densityShader->SetReal("minMass", m_gridMinMass);
	densityShader->SetBool("sharp", m_densitySharp);
	densityShader->SetBool("controlMode", controlMode);
	if (m_mpmAlgorithmEngine->m_algo_code == MpmAlgorithmEngine::MPM_ALGORITHM_CODE::CPP) {
		densityShader->SetInt("currGridSizeX", m_mpmAlgorithmEngine->m_cppChunkX * m_chunks_x);
		densityShader->SetInt("currGridSizeY", m_mpmAlgorithmEngine->m_cppChunkY * m_chunks_y);
		//std::cout << m_mpmAlgorithmEngine->m_cppChunkX * m_chunks_x << ", " << m_mpmAlgorithmEngine->m_cppChunkY * m_chunks_y << std::endl;
	}
	else {
		densityShader->SetInt("currGridSizeX", 32 * m_chunks_x);
		densityShader->SetInt("currGridSizeY", 32 * m_chunks_y);
	}
	densityShader->SetInt("GRID_SIZE_X", m_grid->grid_dim_x);
	densityShader->SetInt("GRID_SIZE_Y", m_grid->grid_dim_y);
	densityShader->SetInt("SCREEN_DIM_X", (int)m_mpmRenderWindow->sim_dimensions.x);
	densityShader->SetInt("SCREEN_DIM_Y", (int)m_mpmRenderWindow->sim_dimensions.y);

	glBindVertexArray(openGLScreen->VAO);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, ssbo);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, 0);
	glBindVertexArray(0);
}

