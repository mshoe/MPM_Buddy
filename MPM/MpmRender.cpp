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
	if (m_viewPointClouds) {
		RenderPointClouds(m_zoomPoint, 1.0, m_mpmRenderWindow, m_pPointCloudShader);
	}
	if (m_viewGrid) {
		RenderGrid(m_zoomPoint, 1.0, m_mpmRenderWindow, m_gridShader);
		if (m_viewGridVector) {
			RenderGrid(m_zoomPoint, 1.0, m_mpmRenderWindow, m_gridShaderVector);
		}
	}
	if (m_viewMarchingSquares) {
		RenderMarchingSquares(m_zoomPoint, 1.0, m_mpmRenderWindow, m_gridShaderMarchingSquares);
	}
	// Render Zoom Border
	if (m_showZoomBorder) {
		m_zoomWindowShader->Use();
		m_zoomWindowShader->SetReal("zoomFactor", m_zoomFactor);
		m_zoomWindowShader->SetVec("zoomPoint", m_zoomPoint);
		m_zoomWindow->RenderLineLoop();
	}

	m_mpmGeometryEngine->Render(m_zoomPoint, 1.0, m_mpmRenderWindow);


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

	RenderGridBorder(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_borderShader);

	m_zoomWindow->UnbindFBO();
}

void mpm::MpmEngine::Render()
{
	glViewport(0, 0, (GLsizei)m_openGLScreen->screen_dimensions.x, (GLsizei)m_openGLScreen->screen_dimensions.y);
	glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
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
	openGLScreen->Render();
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
	//gridShader->SetBool("collectiveNodeGraphics", m_collectiveNodeSelectionGraphics);

	glBindVertexArray(VisualizeVAO);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)(GRID_SIZE_X * GRID_SIZE_Y));
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
	glDrawArrays(GL_POINTS, 0, (GLsizei)(GRID_SIZE_X * GRID_SIZE_Y));
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

