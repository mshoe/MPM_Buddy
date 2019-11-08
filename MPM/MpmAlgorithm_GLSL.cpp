#include "MpmEngine.h"



void mpm::MpmEngine::MpmReset_GLSL()
{
	m_pointCloudMap.clear();
	m_timeStep = 0;
	m_time = 0.0;
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gReset->Use();
	glDispatchCompute(m_chunks_x, m_chunks_y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void mpm::MpmEngine::MpmTimeStep_GLSL(real dt)
{
#ifdef MPM_DEBUG
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();
#endif
	MpmTimeStepP2G_GLSL(dt);

	MpmTimeStepExplicitGridUpdate_GLSL(dt);

	// Here we decide how we do the semi-implicit time step, if we are doing one
	if (m_semi_implicit_CR) {
		MpmTimeStepSemiImplicitCRGridUpdate_GLSL(dt);
	}

	MpmTimeStepG2P_GLSL(dt);

	m_timeStep++;
	m_time += m_dt;
#ifdef MPM_DEBUG
	t2 = high_resolution_clock::now();
	std::cout << "Finished calculating timestep in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
#endif
}

void mpm::MpmEngine::MpmTimeStepP2G_GLSL(real dt)
{
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gReset->Use();
	glDispatchCompute(m_chunks_x, m_chunks_y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	m_p2gScatter->Use();
	m_p2gScatter->SetReal("dt", dt);
	m_p2gScatter->SetInt("CHUNKS_X", m_chunks_x);
	m_p2gScatter->SetInt("CHUNKS_Y", m_chunks_y);
	m_p2gScatter->SetuInt("transferScheme", unsigned int(m_transferScheme));
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		/*m_p2gGather->Use();
		m_p2gGather->SetFloat("dt", dt);
		glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);*/
		//m_p2gScatter->SetInt("selectedNodeI", m_node[0]); // for visualization
		//m_p2gScatter->SetInt("selectedNodeJ", m_node[1]);
		int g2p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(g2p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}
}

void mpm::MpmEngine::MpmTimeStepExplicitGridUpdate_GLSL(real dt)
{
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gUpdate->Use();
	m_gUpdate->SetReal("dt", dt);
	m_gUpdate->SetVec("globalForce", m_globalForce);
	m_gUpdate->SetVec("iMpmMouse", m_mouseMpmRenderScreenGridSpaceFull);
	m_gUpdate->SetReal("drag", m_drag);
	m_gUpdate->SetReal("mousePower", m_mousePower);
	glDispatchCompute(m_chunks_x, m_chunks_y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}



void mpm::MpmEngine::MpmTimeStepG2P_GLSL(real dt)
{

	m_g2pGather->Use();
	m_g2pGather->SetReal("dt", dt);
	m_g2pGather->SetInt("CHUNKS_X", m_chunks_x);
	m_g2pGather->SetInt("CHUNKS_Y", m_chunks_y);
	m_g2pGather->SetuInt("transferScheme", (unsigned int)m_transferScheme);
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {

		//// don't advect fixed point clouds
		if (pointCloudPair.second->fixed)
			continue;

		/*m_g2pGather->SetReal("lam", pointCloudPair.second->parameters.lam);
		m_g2pGather->SetReal("mew", pointCloudPair.second->parameters.mew);
		m_g2pGather->SetReal("crit_c", pointCloudPair.second->parameters.crit_c);
		m_g2pGather->SetReal("crit_s", pointCloudPair.second->parameters.crit_s);
		m_g2pGather->SetReal("hardening", pointCloudPair.second->parameters.hardening);*/
		m_g2pGather->SetuInt("comodel", unsigned int(pointCloudPair.second->comodel));
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		//m_g2pGather->SetBool("fixedPointCloud", pointCloudPair.second->fixed);
		int g2p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(g2p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);
}



void mpm::MpmEngine::CalculatePointCloudVolumes_GLSL(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud)
{
	using namespace std::chrono;

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	std::cout << "Calculating initial volumes for '" << pointCloudID << "'...\n";

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	t1 = high_resolution_clock::now();
	m_p2gCalcVolumes->Use();
	glDispatchCompute(m_chunks_x, m_chunks_y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_g2pCalcVolumes->Use();
	m_g2pCalcVolumes->SetInt("CHUNKS_X", m_chunks_x);
	m_g2pCalcVolumes->SetInt("CHUNKS_Y", m_chunks_y);
	int g2p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
	glDispatchCompute(g2p_workgroups, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	t2 = high_resolution_clock::now();
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);

	std::cout << "Finished calculating initial volumes for '" << pointCloudID << "' in " << duration_cast<duration<real>>(t2 - t1).count() << " seconds.\n";
}


