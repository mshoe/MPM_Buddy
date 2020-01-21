#include "MpmControlEngine.h"


void mpm::MpmControlEngine::Render(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen)
{
	/*if (m_renderTargetPointCloud && m_targetPointCloud != nullptr) {
		m_mpmEngine->RenderPointCloud(m_targetPointCloud, zoomPoint, zoomFactor, openGLScreen, m_mpmEngine->m_pPointCloudShader);
	}*/

	if (m_mpmEngine->m_viewGridDensity) {
		if (m_stcg->targetGrid != nullptr && m_renderTargetPointCloud) {
			m_mpmEngine->RenderDensityField(zoomPoint, zoomFactor, 8, m_stcg->targetGridSsbo, true, openGLScreen, m_gridDensityShader);
		}

		if (m_animateSimStates &&
			!m_stcg->simStates.empty() &&
			int(m_stcg->simStates.size()) > m_currSimState&&
			m_stcg->simStates[m_currSimState]->grid != nullptr)
		{
			control::MapCPUControlGridToGPU(m_stcg->simStates[m_currSimState]->grid, m_stcg->gridSsbo);
			m_currSimState += m_simStateFramesPerFrame;
			if (m_currSimState > int(m_stcg->simStates.size()) - 1) {
				m_currSimState = 0;
				if (!m_animateLoop) {
					m_animateSimStates = false;
				}
			}
		}

		if (m_stcg->controlPointCloud != nullptr && m_renderControlPointCloud) {
			m_mpmEngine->RenderDensityField(zoomPoint, zoomFactor, 8, m_stcg->gridSsbo, true, openGLScreen, m_gridDensityShader);
		}
	}
	else {

		if (m_stcg->targetPointCloud != nullptr && m_renderTargetPointCloud) {

			if (m_renderCircles) {
				RenderControlPointCloudCircles(zoomPoint, zoomFactor, m_stcg->targetPointCloud, m_stcg->targetSsbo);
			}
			if (m_renderPoints) {
				RenderControlPointCloud(zoomPoint, zoomFactor, m_stcg->targetPointCloud, m_stcg->targetSsbo);
			}
		}

		if (m_animateSimStates &&
			!m_stcg->simStates.empty() &&
			int(m_stcg->simStates.size()) > m_currSimState&&
			m_stcg->simStates[m_currSimState]->pointCloud != nullptr)
		{
			control::MapCPUControlPointCloudToGPU(m_stcg->simStates[m_currSimState]->pointCloud, m_stcg->controlSsbo);
			m_currSimState += m_simStateFramesPerFrame;
			if (m_currSimState > int(m_stcg->simStates.size()) - 1) {
				m_currSimState = 0;
				if (!m_animateLoop) {
					m_animateSimStates = false;
				}
			}
		}

		if (m_stcg->controlPointCloud != nullptr && m_renderControlPointCloud) {
			if (m_renderCircles) {
				RenderControlPointCloudCircles(zoomPoint, zoomFactor, m_stcg->controlPointCloud, m_stcg->controlSsbo);
			}
			if (m_renderPoints) {
				RenderControlPointCloud(zoomPoint, zoomFactor, m_stcg->controlPointCloud, m_stcg->controlSsbo);
			}
		}
	}
}

void mpm::MpmControlEngine::RenderControlPointCloud(vec2 zoomPoint, real zoomFactor, std::shared_ptr<control::ControlPointCloud> pointCloud, GLuint ssbo)
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
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, ssbo);
	glDrawArrays(GL_POINTS, 0, (GLsizei)pointCloud->controlPoints.size());
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, 0);
	glBindVertexArray(0);
}

void mpm::MpmControlEngine::RenderControlPointCloudCircles(vec2 zoomPoint, real zoomFactor, std::shared_ptr<control::ControlPointCloud> pointCloud, GLuint ssbo)
{
	m_pRenderControlPointCloudCircles->Use();
	m_pRenderControlPointCloudCircles->SetReal("zoomFactor", zoomFactor);
	m_pRenderControlPointCloudCircles->SetVec("zoomPoint", zoomPoint);
	m_pRenderControlPointCloudCircles->SetReal("pointRadius", m_pointCircleRadius);
	m_pRenderControlPointCloudCircles->SetBool("renderDeformationGradients", m_renderDeformationGradients);
	m_pRenderControlPointCloudCircles->SetBool("renderControlDeformationGradients", m_renderControlDeformationGradients);
	// iResolution and iSourceResolution should be same for the zoom window we make, and iCenter should be the actual center
	//m_pPointCloudShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	//m_pPointCloudShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	//m_pPointCloudShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	//m_pPointCloudShader->SetVec("iCenter", m_openGLScreen->center);
	glBindVertexArray(m_mpmEngine->VisualizeVAO);
	m_pRenderControlPointCloudCircles->SetVec("pointCloudColor", pointCloud->color);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, ssbo);
	glDrawArrays(GL_POINTS, 0, (GLsizei)pointCloud->controlPoints.size());
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, 0);
	glBindVertexArray(0);
}
