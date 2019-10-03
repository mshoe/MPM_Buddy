#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"

void mpm::MpmEngine::SetDeformationGradients(std::string pointCloudID, mat2 Fe, mat2 Fp)
{
	if (m_pointCloudMap.count(pointCloudID)) {
		std::shared_ptr<PointCloud> pointCloud = m_pointCloudMap[pointCloudID];
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
		m_pSetDeformationGradients->Use();
		m_pSetDeformationGradients->SetMat("setFe", Fe);
		m_pSetDeformationGradients->SetMat("setFp", Fp);
		int p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
}

void mpm::MpmEngine::MultiplyDeformationGradients(std::string pointCloudID, mat2 multFe, mat2 multFp)
{
	if (m_pointCloudMap.count(pointCloudID)) {
		std::shared_ptr<PointCloud> pointCloud = m_pointCloudMap[pointCloudID];
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
		m_pMultDeformationGradients->Use();
		m_pMultDeformationGradients->SetMat("multFe", multFe);
		m_pMultDeformationGradients->SetMat("multFp", multFp);
		int p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
}
