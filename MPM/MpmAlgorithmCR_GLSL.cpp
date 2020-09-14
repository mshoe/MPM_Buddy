//#include "MpmAlgorithmEngine.h"
//
//void mpm::MpmAlgorithmEngine::MpmTimeStepSemiImplicitCRGridUpdate_GLSL(real dt)
//{
//	MpmCRInit_GLSL(dt);
//	real L2_norm_rk = 0.0;
//	for (m_cr_step = 0; m_cr_step < m_max_conj_res_iter; m_cr_step++) {
//		bool L2_converged;
//		bool L_inf_converged;
//
//		MpmCRStep_GLSL(dt, L2_norm_rk, L2_converged, L_inf_converged);
//		/*if (L2_converged) {
//			break;
//		}*/
//	}
//	//if (m_cr_step == 0) {
//
//	//}
//	//else {
//	//	if (m_cr_step < m_max_conj_res_iter) {
//	//		std::cout << "CR converged after " << m_cr_step << " steps" << std::endl;
//	//	}
//	//	else {
//	//		std::cout << "CR did not converge after " << m_max_conj_res_iter << " steps." << std::endl;
//	//		m_paused = m_pause_if_not_converged;
//
//	//		// This code is just for finding which node is not convering and doing something to it
//	//		//void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
//	//		//GridNode* data = static_cast<GridNode*>(ptr);
//	//		//for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; i++) {
//	//		//	GridNode gn = data[i];
//	//		//	/*if (!gn.converged) {
//	//		//		converged = false;
//	//		//	}*/
//	//		//	if (gn.m > 0.0) {
//	//		//		real node_norm = glm::length(glm::dot(gn.rk, gn.rk)); // using norm instead of norm squared, cuz more stable
//	//		//		L2_norm_rk += node_norm;
//	//		//		if (node_norm > 0.1) {
//	//		//			if (data[i].m < 0.000001 * m_mpParameters.density) {
//	//		//				int node_i = i / GRID_SIZE_Y;
//	//		//				int node_j = i % GRID_SIZE_Y;
//	//		//				std::cout << "node: (" << node_i << ", " << node_j << ") was too crazy." << std::endl;
//	//		//				std::cout << "explicit guessed velocity: (" << data[i].v.x << ", " << data[i].v.y << ")." << std::endl;
//	//		//				std::cout << "semi-implicit guessed velocity: (" << data[i].xk.x << ", " << data[i].xk.y << ")." << std::endl;
//	//		//				//data[i].xk = vec2(0.0);
//	//		//			}
//	//		//		}
//	//		//	}
//	//		//}
//	//		//glUnmapNamedBuffer(gridSSBO);
//	//	}
//	//	std::cout << "L2 norm of rk is: " << L2_norm_rk << std::endl;
//	//}
//	MpmCREnd_GLSL(dt);
//}
//
//void mpm::MpmAlgorithmEngine::MpmCRInit_GLSL(real dt) {
//	// IMPLICIT INIT PART 1
//	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
//	m_gConjugateResidualsInitPart1->Use();
//	glDispatchCompute(m_mpmEngine->m_chunks_x, m_mpmEngine->m_chunks_y, 1);
//	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
//
//	// CALCULATE deltaForce using v from explicit update
//	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {
//		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
//		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
//		m_p2g2pDeltaForce->Use();
//		m_p2g2pDeltaForce->SetReal("lam", pointCloudPair.second->parameters.lam);
//		m_p2g2pDeltaForce->SetReal("mew", pointCloudPair.second->parameters.mew);
//		m_p2g2pDeltaForce->SetInt("CHUNKS_X", m_mpmEngine->m_chunks_x);
//		m_p2g2pDeltaForce->SetInt("CHUNKS_Y", m_mpmEngine->m_chunks_y);
//		int p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
//		glDispatchCompute(p_workgroups, 1, 1);
//		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
//	}
//
//	// IMPLICIT INIT PART 2
//	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
//	m_gConjugateResidualsInitPart2->Use();
//	m_gConjugateResidualsInitPart2->SetReal("dt", dt);
//	m_gConjugateResidualsInitPart2->SetReal("IMPLICIT_RATIO", m_semi_implicit_ratio);
//	glDispatchCompute(m_mpmEngine->m_chunks_x, m_mpmEngine->m_chunks_y, 1);
//	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
//
//	// CALCULATE deltaForce using r0
//	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {
//		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
//		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
//		m_p2g2pDeltaForce->Use();
//		m_p2g2pDeltaForce->SetReal("lam", pointCloudPair.second->parameters.lam);
//		m_p2g2pDeltaForce->SetReal("mew", pointCloudPair.second->parameters.mew);
//		m_p2g2pDeltaForce->SetInt("CHUNKS_X", m_mpmEngine->m_chunks_x);
//		m_p2g2pDeltaForce->SetInt("CHUNKS_Y", m_mpmEngine->m_chunks_y);
//		int p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
//		glDispatchCompute(p_workgroups, 1, 1);
//		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
//	}
//
//	// IMPLICIT INIT PART 3
//	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
//	m_gConjugateResidualsInitPart3->Use();
//	m_gConjugateResidualsInitPart3->SetReal("dt", dt);
//	m_gConjugateResidualsInitPart3->SetReal("IMPLICIT_RATIO", m_semi_implicit_ratio);
//	glDispatchCompute(m_mpmEngine->m_chunks_x, m_mpmEngine->m_chunks_y, 1);
//	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
//}
//
//bool mpm::MpmAlgorithmEngine::MpmCRStep_GLSL(real dt, real& L2_norm_rk, bool& L2_converged, bool& L_inf_converged)
//{
//	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
//	m_gConjugateResidualsStepPart1->Use();
//	glDispatchCompute(m_mpmEngine->m_chunks_x, m_mpmEngine->m_chunks_y, 1);
//	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
//
//	// now need to calculate the norm of the residual vector to determine if we should stop
//
//	// It is a very bad and slow idea to check if the grid nodes converged through the CPU,
//	// since getting data from the GPU is expensive.
//
//	// Instead, we will use a parallel prefix sum algorithm to check if all grid nodes have converged.
//
//
//	// THIS CODE CHECKS IF CONVERGED ON CPU
//
//	//L2_converged = false;
//	//L_inf_converged = true;
//	//L2_norm_rk = 0.0;
//	//int num_active_nodes = 0;
//	//void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
//	//GridNode* data = static_cast<GridNode*>(ptr);
//	//for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; i++) {
//	//	GridNode gn = data[i];
//	//	/*if (!gn.converged) {
//	//		converged = false;
//	//	}*/
//	//	if (gn.m > 0.0) {
//	//		num_active_nodes++;
//	//		real node_norm = glm::length(glm::dot(gn.rk, gn.rk)); // using norm instead of norm squared, cuz more stable
//	//		L2_norm_rk += node_norm;
//	//	}
//	//}
//	//glUnmapNamedBuffer(gridSSBO);
//
//	////std::cout << "L2 norm of rk (squared) is: " << L2_norm_rk << std::endl;
//
//	//if (num_active_nodes == 0)
//	//	return true;
//
//	//if (L2_norm_rk < m_L2_norm_threshold/* * num_active_nodes*/) {
//	//	L2_converged = true;
//	//}
//
//	//if (L2_converged)
//	//	return L2_converged;
//
//	//if (converged) {
//	//	std::cout << "CR converged after " << i << " steps." << std::endl;
//	//	//break;
//	//}
//
//	// CALCULATE deltaForce using rk
//	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {
//		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
//		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
//		m_p2g2pDeltaForce->Use();
//		m_p2g2pDeltaForce->SetReal("lam", pointCloudPair.second->parameters.lam);
//		m_p2g2pDeltaForce->SetReal("mew", pointCloudPair.second->parameters.mew);
//		m_p2g2pDeltaForce->SetInt("CHUNKS_X", m_mpmEngine->m_chunks_x);
//		m_p2g2pDeltaForce->SetInt("CHUNKS_Y", m_mpmEngine->m_chunks_y);
//		int p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
//		glDispatchCompute(p_workgroups, 1, 1);
//		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
//	}
//
//	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
//	m_gConjugateResidualsStepPart2->Use();
//	m_gConjugateResidualsStepPart2->SetReal("dt", dt);
//	m_gConjugateResidualsStepPart2->SetReal("IMPLICIT_RATIO", m_semi_implicit_ratio);
//	glDispatchCompute(m_mpmEngine->m_chunks_x, m_mpmEngine->m_chunks_y, 1);
//	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
//
//	return L2_converged;
//}
//
//void mpm::MpmAlgorithmEngine::MpmCREnd_GLSL(real dt) {
//
//	// IMPLICIT CONCLUSION
//	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
//	m_gConjugateResidualsConclusion->Use();
//	glDispatchCompute(m_mpmEngine->m_chunks_x, m_mpmEngine->m_chunks_y, 1);
//	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
//}