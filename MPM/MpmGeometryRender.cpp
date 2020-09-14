#include "MpmGeometryEngine.h"

void mpm::MpmGeometryEngine::Render(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen)
{
	if (m_createCircleState) {
		RenderCircle(zoomPoint, 1.0, openGLScreen, m_circleShader);
	}
	if (m_renderPolygonAtMouseState) {
		vec2 mouse = m_mpmEngine->m_mouseMpmRenderScreenGridSpace;
		RenderPolygon(mouse, false, zoomPoint, 1.0, openGLScreen, m_polygonShader);
	}

	if (m_renderPolygon) {
		RenderPolygon(m_polygon->center, m_addPolygonVertexState, zoomPoint, 1.0, openGLScreen, m_polygonShader);
	}

	if (m_renderPWLine) {
		RenderPWLine(zoomPoint, 1.0, openGLScreen, m_pwLineShader);
	}


	// meshes
	m_meshShader->Use();
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	for (std::shared_ptr<Mesh>& mesh : m_meshes) {
		mesh->Draw();
	}
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

}

void mpm::MpmGeometryEngine::RenderCircle(vec2 zoomPoint, real zoomFactor, 
										  std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> circleShader)
{
	vec4 mpmMouse = m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull;
	GLuint vao = m_mpmEngine->VisualizeVAO;
	circleShader->Use();
	circleShader->SetVec("mouse", mpmMouse);
	circleShader->SetReal("radius", m_circle_r);
	circleShader->SetVec("zoomPoint", zoomPoint);
	circleShader->SetReal("zoomFactor", zoomFactor);

	circleShader->SetInt("GRID_SIZE_X", m_mpmEngine->m_grid->grid_dim_x);
	circleShader->SetInt("GRID_SIZE_Y", m_mpmEngine->m_grid->grid_dim_y);
	circleShader->SetInt("SCREEN_DIM_X", (int)m_mpmEngine->m_mpmRenderWindow->sim_dimensions.x);
	circleShader->SetInt("SCREEN_DIM_Y", (int)m_mpmEngine->m_mpmRenderWindow->sim_dimensions.y);

	glBindVertexArray(vao);
	glDrawArrays(GL_POINTS, 0, (GLsizei)1);
	glBindVertexArray(0);
}

void mpm::MpmGeometryEngine::RenderPolygon(vec2 polygonCenter, bool addPolygonVertexState, 
										   vec2 zoomPoint, real zoomFactor, 
										   std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> polygonShader)
{
	if (m_polygon->vertices.size() == 0)
		return;
	vec4 mpmMouse = m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull;
	GLuint vao = m_mpmEngine->VisualizeVAO;

	polygonShader->Use();
	polygonShader->SetVec("mouse", mpmMouse);
	polygonShader->SetBool("lastVertexMouse", addPolygonVertexState);

	polygonShader->SetVec("zoomPoint", zoomPoint);
	polygonShader->SetReal("zoomFactor", zoomFactor);
	polygonShader->SetVec("polygonCenter", polygonCenter);

	polygonShader->SetInt("GRID_SIZE_X", m_mpmEngine->m_grid->grid_dim_x);
	polygonShader->SetInt("GRID_SIZE_Y", m_mpmEngine->m_grid->grid_dim_y);
	polygonShader->SetInt("SCREEN_DIM_X", (int)m_mpmEngine->m_mpmRenderWindow->sim_dimensions.x);
	polygonShader->SetInt("SCREEN_DIM_Y", (int)m_mpmEngine->m_mpmRenderWindow->sim_dimensions.y);
	GLuint polygonSSBO;
	glCreateBuffers(1, &polygonSSBO);

	glNamedBufferStorage(
		polygonSSBO,
		sizeof(vec2) * m_polygon->vertices.size(),
		&(m_polygon->vertices.front()),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // adding write bit for debug purposes
	);
	glBindVertexArray(vao);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, polygonSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)1);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, &polygonSSBO);
}

void mpm::MpmGeometryEngine::RenderPWLine(vec2 zoomPoint, real zoomFactor, 
										  std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pwLineShader)
{
	if (m_pwLine->vertices.size() == 0)
		return;

	vec4 mpmMouse = m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull;
	GLuint vao = m_mpmEngine->VisualizeVAO;

	pwLineShader->Use();
	pwLineShader->SetVec("mouse", mpmMouse);
	pwLineShader->SetBool("lastVertexMouse", m_addPWLineVertexState);

	pwLineShader->SetVec("zoomPoint", zoomPoint);
	pwLineShader->SetReal("zoomFactor", zoomFactor);

	pwLineShader->SetInt("GRID_SIZE_X", m_mpmEngine->m_grid->grid_dim_x);
	pwLineShader->SetInt("GRID_SIZE_Y", m_mpmEngine->m_grid->grid_dim_y);
	pwLineShader->SetInt("SCREEN_DIM_X", (int)m_mpmEngine->m_mpmRenderWindow->sim_dimensions.x);
	pwLineShader->SetInt("SCREEN_DIM_Y", (int)m_mpmEngine->m_mpmRenderWindow->sim_dimensions.y);

	GLuint pwLineSSBO;
	glCreateBuffers(1, &pwLineSSBO);

	glNamedBufferStorage(
		pwLineSSBO,
		sizeof(vec2) * m_pwLine->vertices.size(),
		&(m_pwLine->vertices.front()),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // adding write bit for debug purposes
	);
	glBindVertexArray(vao);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, pwLineSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)1);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, &pwLineSSBO);
}

void mpm::MpmGeometryEngine::SetSelectionUniforms(std::shared_ptr<StandardShader> pointCloudShader)
{
	pointCloudShader->SetBool("visualizeSelected", m_visualizeSelected);
	pointCloudShader->SetVec("pointSelectColor", m_pointSelectColor);
}