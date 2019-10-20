#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"

void mpm::MpmEngine::Render()
{
	glViewport(0, 0, (GLsizei)m_openGLScreen->screen_dimensions.x, (GLsizei)m_openGLScreen->screen_dimensions.y);
	glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
	glClear(GL_COLOR_BUFFER_BIT);

	// RENDER MOUSE SHADER
	RenderScreenShader(m_zoomPoint, 1.0, m_openGLScreen);

	// Render material point clouds
	if (m_viewPointClouds) {
		RenderPointClouds(m_zoomPoint, 1.0, m_openGLScreen, m_pPointCloudShader);
	}

	// Render grid nodes
	if (m_viewGrid) {
		RenderGrid(m_zoomPoint, 1.0, m_openGLScreen, m_gridShader);
		if (m_viewGridVector) {
			RenderGrid(m_zoomPoint, 1.0, m_openGLScreen, m_gridShaderVector);
		}
	}

	// Render marching squares
	if (m_viewMarchingSquares) {
		RenderMarchingSquares(m_zoomPoint, 1.0, m_openGLScreen, m_gridShaderMarchingSquares);
	}

	// Render Zoom Border
	if (m_showZoomBorder) {
		m_zoomWindowShader->Use();
		m_zoomWindowShader->SetVec("iSourceResolution", m_openGLScreen->screen_dimensions);
		m_zoomWindowShader->SetVec("iResolution", m_openGLScreen->sim_dimensions);
		m_zoomWindowShader->SetVec("iCenter", vec2(m_openGLScreen->center.x, m_openGLScreen->screen_dimensions.y - m_openGLScreen->center.y)); // correct y for glsl
		m_zoomWindowShader->SetReal("zoomFactor", m_zoomFactor);
		m_zoomWindowShader->SetVec("zoomPoint", m_zoomPoint);

		m_zoomWindow->RenderLineLoop();
	}



	// Render polygon
	if (m_renderPolygon) {
		RenderPolygon(m_zoomPoint, m_zoomFactor, m_openGLScreen, m_polygonShader);
	}

	if (m_renderPWLine) {
		RenderPWLine(m_zoomPoint, m_zoomFactor, m_openGLScreen, m_pwLineShader);
	}

	// Render the grid border
	m_borderShader->Use();
	m_borderShader->SetVec("iCenter", vec2(m_openGLScreen->center.x, m_openGLScreen->screen_dimensions.y - m_openGLScreen->center.y)); // correct y for glsl
	m_borderShader->SetVec("iResolution", m_openGLScreen->sim_dimensions);
	m_borderShader->SetVec("iSourceResolution", m_openGLScreen->screen_dimensions);
	m_borderShader->SetInt("CHUNKS_X", m_chunks_x);
	m_borderShader->SetInt("CHUNKS_Y", m_chunks_y);
	glBindVertexArray(VisualizeVAO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)1);
	glBindVertexArray(0);

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
	m_zoomWindow->UnbindFBO();

}

void mpm::MpmEngine::RenderScreenShader(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen)
{
	m_mouseShader->Use();
	m_mouseShader->SetVec("iMouse", m_mouse);
	m_mouseShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	m_mouseShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	m_mouseShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
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
	m_pPointCloudShader->Use();
	m_pPointCloudShader->SetReal("maxSpeedClamp", m_maxSpeedClamp);
	m_pPointCloudShader->SetReal("minSpeedClamp", m_minSpeedClamp);
	m_pPointCloudShader->SetBool("visualizeSpeed", m_visualizeSpeed);
	m_pPointCloudShader->SetReal("maxEnergyClamp", m_maxEnergyClamp);
	m_pPointCloudShader->SetReal("minEnergyClamp", m_minEnergyClamp);
	m_pPointCloudShader->SetBool("visualizeEnergy", m_visualizeEnergy);
	m_pPointCloudShader->SetReal("zoomFactor", zoomFactor);
	m_pPointCloudShader->SetVec("zoomPoint", zoomPoint);
	// iResolution and iSourceResolution should be same for the zoom window we make, and iCenter should be the actual center
	m_pPointCloudShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	m_pPointCloudShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	m_pPointCloudShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	//m_pPointCloudShader->SetVec("iCenter", m_openGLScreen->center);
	glBindVertexArray(VisualizeVAO);
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		m_pPointCloudShader->SetVec("pointCloudColor", pointCloudPair.second->color);
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
	gridShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	gridShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	gridShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
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
	gridShader->SetReal("dt", m_dt);
	gridShader->SetInt("selectedVector", m_gridVectorOption);
	gridShader->SetReal("maxGridVectorLength", m_maxGridVectorLength);
	gridShader->SetReal("maxGridVectorVisualLength", m_maxGridVectorVisualLength);
	gridShader->SetBool("collectiveNodeGraphics", m_collectiveNodeSelectionGraphics);

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
	// iResolution and iSourceResolution should be same for the zoom window we make, and iCenter should be the actual center
	gridShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	gridShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	gridShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	gridShader->SetReal("isoMass", m_isoMass);
	gridShader->SetVec("mscolor", m_marchingSquaresColor);
	gridShader->SetReal("dt", m_dt);

	glBindVertexArray(VisualizeVAO);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)(GRID_SIZE_X * GRID_SIZE_Y));
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderPolygon(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> polygonShader)
{
	if (m_polygon->vertices.size() == 0)
		return;

	polygonShader->Use();
	polygonShader->SetVec("iMouse", m_mpm_mouse);
	polygonShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	polygonShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	polygonShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	polygonShader->SetBool("lastVertexMouse", m_addPolygonVertexState);
	GLuint polygonSSBO;
	glCreateBuffers(1, &polygonSSBO);

	glNamedBufferStorage(
		polygonSSBO,
		sizeof(vec2) * m_polygon->vertices.size(),
		&(m_polygon->vertices.front()),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // adding write bit for debug purposes
	);
	glBindVertexArray(VisualizeVAO);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, polygonSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)1);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, &polygonSSBO);
}

void mpm::MpmEngine::RenderPWLine(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pwLineShader)
{
	if (m_pwLine->vertices.size() == 0)
		return;

	pwLineShader->Use();
	pwLineShader->SetVec("iMouse", m_mpm_mouse);
	pwLineShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	pwLineShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	pwLineShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	pwLineShader->SetBool("lastVertexMouse", m_addPWLineVertexState);
	GLuint pwLineSSBO;
	glCreateBuffers(1, &pwLineSSBO);

	glNamedBufferStorage(
		pwLineSSBO,
		sizeof(vec2) * m_pwLine->vertices.size(),
		&(m_pwLine->vertices.front()),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // adding write bit for debug purposes
	);
	glBindVertexArray(VisualizeVAO);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, pwLineSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)1);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, &pwLineSSBO);
}