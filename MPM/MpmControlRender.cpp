#include "MpmControlEngine.h"


void mpm::MpmControlEngine::Render(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen)
{
	/*if (m_renderTargetPointCloud && m_targetPointCloud != nullptr) {
		m_mpmEngine->RenderPointCloud(m_targetPointCloud, zoomPoint, zoomFactor, openGLScreen, m_mpmEngine->m_pPointCloudShader);
	}*/


	if (m_targetPointCloud != nullptr && m_renderTargetPointCloud) {
		RenderControlPointCloud(zoomPoint, zoomFactor, m_targetPointCloud);
	}

	if (m_controlPointCloud != nullptr) {
		RenderControlPointCloud(zoomPoint, zoomFactor, m_controlPointCloud);
	}
}

void mpm::MpmControlEngine::RenderControlPointCloud(vec2 zoomPoint, real zoomFactor, std::shared_ptr<control::ControlPointCloud> pointCloud)
{
	m_pRenderControlPointCloud->Use();
	m_pRenderControlPointCloud->SetReal("zoomFactor", zoomFactor);
	m_pRenderControlPointCloud->SetVec("zoomPoint", zoomPoint);

	// iResolution and iSourceResolution should be same for the zoom window we make, and iCenter should be the actual center
	//m_pPointCloudShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	//m_pPointCloudShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	//m_pPointCloudShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	//m_pPointCloudShader->SetVec("iCenter", m_openGLScreen->center);
	glBindVertexArray(m_mpmEngine->VisualizeVAO);
	m_pRenderControlPointCloud->SetVec("pointCloudColor", pointCloud->color);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, pointCloud->ssbo);
	glDrawArrays(GL_POINTS, 0, (GLsizei)pointCloud->controlPoints.size());
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, 0);
	glBindVertexArray(0);
}