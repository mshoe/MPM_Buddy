#include "MpmEngine.h"

void mpm::MpmEngine::MpmTimeStep(real dt)
{
#ifdef MPM_DEBUG
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();
#endif
	MpmTimeStepP2G(dt);

	MpmTimeStepExplicitGridUpdate(dt);

	if (m_implicit) {
		MpmTimeStepSemiImplicitGridUpdate(dt);
	}

	MpmTimeStepG2P(dt);
#ifdef MPM_DEBUG
	t2 = high_resolution_clock::now();
	std::cout << "Finished calculating timestep in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
#endif
}

void mpm::MpmEngine::MpmTimeStepP2G(real dt)
{
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gReset->Use();
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		/*m_p2gGather->Use();
		m_p2gGather->SetFloat("dt", dt);
		glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);*/
		m_p2gScatter->Use();
		m_p2gScatter->SetReal("dt", dt);
		//m_p2gScatter->SetInt("selectedNodeI", m_node[0]); // for visualization
		//m_p2gScatter->SetInt("selectedNodeJ", m_node[1]);
		int g2p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(g2p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}
}

void mpm::MpmEngine::MpmTimeStepExplicitGridUpdate(real dt)
{
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gUpdate->Use();
	m_gUpdate->SetReal("dt", dt);
	m_gUpdate->SetVec("globalForce", m_globalForce);
	m_gUpdate->SetVec("iMpmMouse", m_mpm_mouse);
	m_gUpdate->SetReal("drag", m_drag);
	m_gUpdate->SetReal("mousePower", m_mousePower);
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

void mpm::MpmEngine::MpmTimeStepSemiImplicitGridUpdate(real dt)
{
	MpmCRInit(dt);
	real L2_norm_rk = 0.0;
	for (m_cr_step = 0; m_cr_step < m_max_conj_res_iter; m_cr_step++) {
		bool L2_converged;
		bool L_inf_converged;
		
		MpmCRStep(dt, L2_norm_rk, L2_converged, L_inf_converged);
		if (L2_converged) {
			break;
		}
	}
	if (m_cr_step == 0) {
		
	}
	else {
		if (m_cr_step < m_max_conj_res_iter) {
			std::cout << "CR converged after " << m_cr_step << " steps" << std::endl;
		}
		else {
			std::cout << "CR did not converge after " << m_max_conj_res_iter << " steps." << std::endl;
			m_paused = m_pause_if_not_converged;
			void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
			GridNode* data = static_cast<GridNode*>(ptr);
			for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; i++) {
				GridNode gn = data[i];
				/*if (!gn.converged) {
					converged = false;
				}*/
				if (gn.m > 0.0) {
					real node_norm = glm::length(glm::dot(gn.rk, gn.rk)); // using norm instead of norm squared, cuz more stable
					L2_norm_rk += node_norm;
					if (node_norm > 0.1) {
						if (data[i].m < 0.000001 * m_mpParameters.density) {
							int node_i = i / GRID_SIZE_Y;
							int node_j = i % GRID_SIZE_Y;
							std::cout << "node: (" << node_i << ", " << node_j << ") was too crazy." << std::endl;
							std::cout << "explicit guessed velocity: (" << data[i].v.x << ", " << data[i].v.y << ")." << std::endl;
							std::cout << "semi-implicit guessed velocity: (" << data[i].xk.x << ", " << data[i].xk.y << ")." << std::endl;
							//data[i].xk = vec2(0.0);
						}
					}
				}
			}
			glUnmapNamedBuffer(gridSSBO);
		}
		std::cout << "L2 norm of rk is: " << L2_norm_rk << std::endl;
	}
	MpmCREnd(dt);
}

void mpm::MpmEngine::MpmTimeStepG2P(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		m_g2pGather->Use();
		m_g2pGather->SetReal("dt", dt);
		m_g2pGather->SetReal("lam", pointCloudPair.second->lam);
		m_g2pGather->SetReal("mew", pointCloudPair.second->mew);
		m_g2pGather->SetReal("crit_c", pointCloudPair.second->parameters.crit_c);
		m_g2pGather->SetReal("crit_s", pointCloudPair.second->parameters.crit_s);
		m_g2pGather->SetReal("hardening", pointCloudPair.second->parameters.hardening);
		m_g2pGather->SetuInt("comodel", pointCloudPair.second->comodel);
		int g2p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(g2p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);
	m_timeStep++;
	m_time += dt;
}

void mpm::MpmEngine::MpmCRInit(real dt) {
	// IMPLICIT INIT PART 1
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsInitPart1->Use();
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	// CALCULATE deltaForce using v from explicit update
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		m_p2g2pDeltaForce->Use();
		m_p2g2pDeltaForce->SetReal("lam", pointCloudPair.second->lam);
		m_p2g2pDeltaForce->SetReal("mew", pointCloudPair.second->mew);
		int p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}

	// IMPLICIT INIT PART 2
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsInitPart2->Use();
	m_gConjugateResidualsInitPart2->SetReal("dt", dt);
	m_gConjugateResidualsInitPart2->SetReal("IMPLICIT_RATIO", m_implicit_ratio);
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	// CALCULATE deltaForce using r0
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		m_p2g2pDeltaForce->Use();
		m_p2g2pDeltaForce->SetReal("lam", pointCloudPair.second->lam);
		m_p2g2pDeltaForce->SetReal("mew", pointCloudPair.second->mew);
		int p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}

	// IMPLICIT INIT PART 3
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsInitPart3->Use();
	m_gConjugateResidualsInitPart3->SetReal("dt", dt);
	m_gConjugateResidualsInitPart3->SetReal("IMPLICIT_RATIO", m_implicit_ratio);
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}

bool mpm::MpmEngine::MpmCRStep(real dt, real &L2_norm_rk, bool &L2_converged, bool &L_inf_converged)
{
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsStepPart1->Use();
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	// now need to calculate the norm of the residual vector to determine if we should stop

	// It is a very bad and slow idea to check if the grid nodes converged through the CPU,
	// since getting data from the GPU is expensive.

	// Instead, we will use a parallel prefix sum algorithm to check if all grid nodes have converged.

	L2_converged = false;
	L_inf_converged = true;
	L2_norm_rk = 0.0;
	int num_active_nodes = 0;
	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	GridNode* data = static_cast<GridNode*>(ptr);
	for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; i++) {
		GridNode gn = data[i];
		/*if (!gn.converged) {
			converged = false;
		}*/
		if (gn.m > 0.0) {
			num_active_nodes++;
			real node_norm = glm::length(glm::dot(gn.rk, gn.rk)); // using norm instead of norm squared, cuz more stable
			L2_norm_rk += node_norm;
		}
	}
	glUnmapNamedBuffer(gridSSBO);

	//std::cout << "L2 norm of rk (squared) is: " << L2_norm_rk << std::endl;

	if (num_active_nodes == 0)
		return true;

	if (L2_norm_rk < m_L2_norm_threshold/* * num_active_nodes*/) {
		L2_converged = true;
	}

	if (L2_converged)
		return L2_converged;

	//if (converged) {
	//	std::cout << "CR converged after " << i << " steps." << std::endl;
	//	//break;
	//}

	// CALCULATE deltaForce using rk
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		m_p2g2pDeltaForce->Use();
		m_p2g2pDeltaForce->SetReal("lam", pointCloudPair.second->lam);
		m_p2g2pDeltaForce->SetReal("mew", pointCloudPair.second->mew);
		int p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsStepPart2->Use();
	m_gConjugateResidualsStepPart2->SetReal("dt", dt);
	m_gConjugateResidualsStepPart2->SetReal("IMPLICIT_RATIO", m_implicit_ratio);
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	return L2_converged;
}

void mpm::MpmEngine::MpmCREnd(real dt) {

	// IMPLICIT CONCLUSION
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsConclusion->Use();
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
}


void mpm::MpmEngine::CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud)
{
	using namespace std::chrono;

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	std::cout << "Calculating initial volumes for '" << pointCloudID << "'...\n";

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	t1 = high_resolution_clock::now();
	m_p2gCalcVolumes->Use();
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_g2pCalcVolumes->Use();
	int g2p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
	glDispatchCompute(g2p_workgroups, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	t2 = high_resolution_clock::now();
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);

	std::cout << "Finished calculating initial volumes for '" << pointCloudID << "' in " << duration_cast<duration<real>>(t2 - t1).count() << " seconds.\n";
}

void mpm::MpmEngine::SetReferenceConfig(std::string pointCloudID)
{
	if (m_pointCloudMap.count(pointCloudID)) {
		std::shared_ptr<PointCloud> pointCloud = m_pointCloudMap[pointCloudID];
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
		m_pSetReferenceConfig->Use();
		int p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
}
