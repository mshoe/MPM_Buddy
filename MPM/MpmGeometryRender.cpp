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

	glBindVertexArray(vao);
	glDrawArrays(GL_POINTS, 0, (GLsizei)1);
	glBindVertexArray(0);

	circleShader->Use();
	circleShader->SetVec("mouse", mpmMouse);
	circleShader->SetReal("radius", m_circle_inner_radius);
	circleShader->SetVec("zoomPoint", zoomPoint);
	circleShader->SetReal("zoomFactor", zoomFactor);

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