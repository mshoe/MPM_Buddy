#include "MpmGeometryEngine.h"

void mpm::MpmGeometryEngine::SelectPointsInPolygon(std::string pointCloudID)
{
	if (m_mpmEngine->m_pointCloudMap.count(pointCloudID)) {
		//std::shared_ptr<PointCloud> pointCloud = m_mpmEngine->m_pointCloudMap[pointCloudID];
		//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);

		//GLuint polygonSSBO;
		//glCreateBuffers(1, &polygonSSBO);

		//glNamedBufferStorage(
		//	polygonSSBO,
		//	sizeof(vec2) * m_polygon->vertices.size(),
		//	&(m_polygon->vertices.front()),
		//	GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // adding write bit for debug purposes
		//);
		//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, polygonSSBO);

		//m_pLassoTool->Use();
		//m_pLassoTool->SetVec("polygonCenter", m_polygon->center);
		//int p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
		//glDispatchCompute(p_workgroups, 1, 1);
		//glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
		//glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, 0);
		//glDeleteBuffers(1, &polygonSSBO);
	}
}

void mpm::MpmGeometryEngine::ClearPointSelection(std::string pointCloudID)
{
	if (m_mpmEngine->m_pointCloudMap.count(pointCloudID)) {
		/*std::shared_ptr<PointCloud> pointCloud = m_mpmEngine->m_pointCloudMap[pointCloudID];
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);


		m_pClearPointSelection->Use();
		int p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);*/
	}
}


