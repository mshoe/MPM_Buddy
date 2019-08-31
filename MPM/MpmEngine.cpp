#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"

real BSpline(real x) {
	return (x < 0.5) ? glm::step(0.0, x)*(0.75 - x * x) :
		glm::step(x, 1.5)*0.5*(1.5 - abs(x))*(1.5 - abs(x));
}

real BSplineSlope(real x) {
	return (x < 0.5) ? glm::step(0.0, x)*(-2.0 * x) :
		glm::step(x, 1.5)*(1.5 - abs(x))*x / abs(x);
}



bool mpm::MpmEngine::InitComputeShaderPipeline()
{
	using namespace std::chrono;

	glEnable(GL_PROGRAM_POINT_SIZE);

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	int work_group_size;
	glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &work_group_size);
	std::cout << "max work group size: " << work_group_size << std::endl;

	// first compile shaders
	m_gReset = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\gResetNodes.comp"}, "shaders\\compute\\mpm_header.comp");
	m_p2gScatter = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\p2gScatterParticleAndUpdateNodes.comp"}, "shaders\\compute\\mpm_header.comp");
	//m_p2gGather = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\p2gGatherParticlesAndUpdateNode.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gUpdate = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\gUpdateNodes.comp"}, "shaders\\compute\\mpm_header.comp");

	m_g2pGather = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\g2pGatherNodesAndUpdateParticle.comp"}, "shaders\\compute\\mpm_header.comp");
	m_p2gCalcVolumes = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\p2gCalculateVolumes.comp"}, "shaders\\compute\\mpm_header.comp");
	m_g2pCalcVolumes = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\g2pCalculateVolumes.comp"}, "shaders\\compute\\mpm_header.comp");

	// implict time integration shaders
	m_p2g2pDeltaForce = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\g2p2gDeltaForce.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsInitPart1 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_InitPart1.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsInitPart2 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_InitPart2.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsInitPart3 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_InitPart3.comp"}, "shaders\\compute\\mpm_header.comp");

	m_gConjugateResidualsStepPart1 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_StepPart1.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsStepPart2 = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_StepPart2.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gConjugateResidualsConclusion = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\implicit\\gCR_Conclusion.comp"}, "shaders\\compute\\mpm_header.comp");

	glCreateVertexArrays(1, &VisualizeVAO);

	m_pPointCloudShader = std::make_unique<StandardShader>(std::vector<std::string>{"shaders\\graphics\\pointCloud.vs"}, std::vector<std::string>{"shaders\\graphics\\pointCloud.fs"}, "shaders\\compute\\mpm_header.comp");
	m_mouseShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\mouseShader.vs"}, std::vector<std::string>{"shaders\\graphics\\mouseShader.fs"}, "shaders\\compute\\mpm_header.comp");
	//m_nodeShader = std::make_shared<StandardShader>(std::vector<std::string>{"shaders\\graphics\\nodeSelectionShader.vs"}, std::vector<std::string>{"shaders\\graphics\\nodeSelectionShader.fs"}, "shaders\\compute\\mpm_header.comp");


	// Initialize the grid SSBO on the GPU
	m_grid = Grid(GRID_SIZE_X, GRID_SIZE_Y);

	glCreateBuffers(1, &gridSSBO);

	glNamedBufferStorage(
		gridSSBO,
		sizeof(GridNode) * GRID_SIZE_X * GRID_SIZE_Y,
		&(m_grid.nodes[0].m),
		GL_MAP_READ_BIT
	);

	//CreateDemo();

	// initialize material parameters here for now
	m_fixedCorotatedParameters.youngMod = 90000.0;
	m_fixedCorotatedParameters.poisson = 0.3;
	m_fixedCorotatedParameters.particleSpacing = 0.25;
	m_fixedCorotatedParameters.density = 40;

	m_simpleSnowParameters.youngMod = 140000.0;
	m_simpleSnowParameters.poisson = 0.2;
	m_simpleSnowParameters.particleSpacing = 0.25;
	m_simpleSnowParameters.density = 40;
	m_simpleSnowParameters.crit_c = 0.025;
	m_simpleSnowParameters.crit_s = 0.0075;
	m_simpleSnowParameters.hardening = 10.0;

	m_mpParameters = m_fixedCorotatedParameters;
	return true;
}

bool mpm::MpmEngine::CleanupComputeShaderPipeline()
{
	m_pointCloudMap.clear();
	glDeleteBuffers(1, &gridSSBO);
	glDeleteVertexArrays(1, &VisualizeVAO);

	return false;
}

void mpm::MpmEngine::MpmTimeStep(real dt)
{
#ifdef MPM_DEBUG
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();
#endif
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
		int g2p_workgroups = int(glm::ceil(real(pointCloudPair.second->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(g2p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gUpdate->Use();
	m_gUpdate->SetReal("dt", dt);
	m_gUpdate->SetVec("globalForce", m_globalForce);
	m_gUpdate->SetVec("mousePos", m_mousePos);
	m_gUpdate->SetReal("drag", m_drag);
	if (m_rightButtonDown)
		m_gUpdate->SetReal("mousePower", m_mousePower);
	else
		m_gUpdate->SetReal("mousePower", 0.0);
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	if (m_implicit) {
		MpmImplictTimeCR(dt);
	}

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
#ifdef MPM_DEBUG
	t2 = high_resolution_clock::now();
	std::cout << "Finished calculating timestep in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
#endif
}

void mpm::MpmEngine::MpmImplictTimeCR(real dt)
{
#ifdef MPM_CR_DEBUG
	PrintGridData();
#endif
	// IMPLICIT INIT PART 1
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsInitPart1->Use();
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

#ifdef MPM_CR_DEBUG
	PrintGridData();
#endif

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

#ifdef MPM_CR_DEBUG
	PrintGridData();
#endif

	// IMPLICIT INIT PART 2
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsInitPart2->Use();
	m_gConjugateResidualsInitPart2->SetReal("dt", dt);
	m_gConjugateResidualsInitPart2->SetReal("IMPLICIT_RATIO", m_implicit_ratio);
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

#ifdef MPM_CR_DEBUG
	PrintGridData();
#endif

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

#ifdef MPM_CR_DEBUG
	PrintGridData();
#endif

	// IMPLICIT INIT PART 3
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsInitPart3->Use();
	m_gConjugateResidualsInitPart3->SetReal("dt", dt);
	m_gConjugateResidualsInitPart3->SetReal("IMPLICIT_RATIO", m_implicit_ratio);
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

#ifdef MPM_CR_DEBUG
	PrintGridData();
#endif

	// Now start the conjugate residuals loop
	for (int i = 0; i < m_max_conj_res_iter; i++) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		m_gConjugateResidualsStepPart1->Use();
		glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

#ifdef MPM_CR_DEBUG
		PrintGridData();
#endif

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

#ifdef MPM_CR_DEBUG
		PrintGridData();
#endif

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		m_gConjugateResidualsStepPart2->Use();
		m_gConjugateResidualsStepPart2->SetReal("dt", dt);
		m_gConjugateResidualsStepPart2->SetReal("IMPLICIT_RATIO", m_implicit_ratio);
		glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

#ifdef MPM_CR_DEBUG
		PrintGridData();
#endif
	}

	// IMPLICIT CONCLUSION
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsConclusion->Use();
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

#ifdef MPM_CR_DEBUG
	PrintGridData();
#endif
}

void mpm::MpmEngine::Render()
{
	if (!m_paused) {
		if (!m_rt) {
			MpmTimeStep(m_dt);
		}
		else {
			real curr_dt = 0.0;
			real rt_dt = 1.0 / 60.0;
			while (curr_dt < rt_dt) {
				MpmTimeStep(m_dt);
				curr_dt += m_dt;
			}
		}
	}
	m_pPointCloudShader->Use();
	glBindVertexArray(VisualizeVAO);
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		m_pPointCloudShader->SetVec("pointCloudColor", pointCloudPair.second->color);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glDrawArrays(GL_POINTS, 0, (GLsizei)pointCloudPair.second->N);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderGUI()
{
	if (m_renderGUI) {
		ImGui::Begin("MPM Grid Data", &m_renderGUI);

		
		ImGui::InputReal("drag", &m_drag, 0.0001, 0.01, "%.4f");

		if (ImGui::Button("Set global force")) {
			m_globalForce.x = m_globalForceArray[0];
			m_globalForce.y = m_globalForceArray[1];
		}
		ImGui::InputReal("Global Force x", &m_globalForce.x, 0.1, 1.0, "%.16f");
		ImGui::InputReal("Global Force y", &m_globalForce.y, 0.1, 1.0, "%.16f");

		ImGui::InputReal("Mouse power", &m_mousePower);

		ImGui::Text(std::to_string(m_time).c_str());
		ImGui::Text(std::to_string(m_timeStep).c_str());
		ImGui::InputReal("dt", &m_dt, 0.001, 1.0/60.0, "%.6f");

		ImGui::Checkbox("Realtime Rendering", &m_rt);
		
		
		ImGui::Checkbox("Node Selection Graphics", &m_nodeGraphicsActive);
		if (ImGui::Button("Get node data") && m_paused) {
			UpdateNodeData();
		}
		ImGui::InputInt2("Grid Node:", m_node);
		ImGui::Text(m_nodeText.c_str());
		
		ImGui::Checkbox("Implicit Time Integration", &m_implicit);
		ImGui::InputReal("Implict Ratio", &m_implicit_ratio);

		if (ImGui::Button("Pause")) {
			m_paused = !m_paused;
		}
		if (ImGui::Button("Advance") && m_paused) {
			MpmTimeStep(m_dt);
			UpdateNodeData();
		}


		if (ImGui::Button("Restart")) {
			//CreateDemo();
			m_pointCloudMap.clear();
			m_timeStep = 0;
			m_time = 0.0;
		}

		ImGui::End();

		ImGui::Begin("Geometry Editor");

		//ImGui::Color
		ImGui::ColorEdit4("Color", m_color);
		//ImGui::InputInt3("Color", m_color);

		switch (m_comodel) {
		case FIXED_COROTATIONAL_ELASTICITY:
			ImGui::InputReal("Young's Modulus", &m_mpParameters.youngMod, 1.0, 10.0, "%.1f");
			ImGui::InputReal("Poisson's Ratio", &m_mpParameters.poisson, 0.005, 0.05, "%.3f");
			ImGui::InputReal("Point Spacing", &m_mpParameters.particleSpacing, 0.01, 0.1, "%.2f");
			ImGui::InputReal("Density", &m_mpParameters.density, 0.01, 0.1, "%.2f");
			break;
		case SIMPLE_SNOW:
			ImGui::InputReal("Young's Modulus", &m_mpParameters.youngMod, 1.0, 10.0, "%.1f");
			ImGui::InputReal("Poisson's Ratio", &m_mpParameters.poisson, 0.005, 0.05, "%.3f");
			ImGui::InputReal("Point Spacing", &m_mpParameters.particleSpacing, 0.01, 0.1, "%.2f");
			ImGui::InputReal("Density", &m_mpParameters.density, 0.01, 0.1, "%.2f");
			ImGui::InputReal("Critical Compression", &m_mpParameters.crit_c, 0.001, 0.01, "%.4f");
			ImGui::InputReal("Critical Stretch", &m_mpParameters.crit_s, 0.001, 0.01, "%.4f");
			ImGui::InputReal("Hardening", &m_mpParameters.hardening, 0.001, 0.01, "%.4f");
			break;
		default:
			break;
		}
		

		if (ImGui::Button("Fixed Corotated Elasticity")) {
			ChangeMaterialParameters(FIXED_COROTATIONAL_ELASTICITY);
		}
		if (ImGui::Button("Stovakhim Snow (2013)")) {
			ChangeMaterialParameters(SIMPLE_SNOW);
		}
		std::string comodelStr = "constitutive model: " + std::to_string(m_comodel);
		ImGui::Text(comodelStr.c_str());



		ImGui::InputReal("Circle Radius", &m_circle_r, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Circle Inner Radius", &m_circle_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Circle Rounding", &m_circle_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Circle") && m_paused) {
			m_createCircleState = true;
		}


		ImGui::InputReal("Rectangle Base Length", &m_rect_b, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Height Length", &m_rect_h, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Inner Radius", &m_rect_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Rounding", &m_rect_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Rectangle") && m_paused) {
			m_createRectState = true;
		}

		ImGui::InputReal("Isosceles Triangle Base Length", &m_iso_tri_b, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Height Length", &m_iso_tri_h, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Triangle Inner Radius", &m_iso_tri_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Triangle Rounding", &m_iso_tri_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Triangle") && m_paused) {
			m_createIsoTriState = true;
		}



		ImGui::End();

		ImGui::Begin("Material Point View");


		
		if (ImGui::CollapsingHeader("Point Clouds")) {
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
				ImGui::Text(pointCloudPair.first.c_str());
			}
		}
		m_pointCloudSelect.resize(30);
		ImGui::InputText("Check point cloud", m_pointCloudSelect.data(), 30);
		std::string pointCloudSelectStr = std::string(m_pointCloudSelect.data());

		if (ImGui::Button("View Particles") && m_paused) {
			if (m_pointCloudMap.count(pointCloudSelectStr)) {
				void* ptr = glMapNamedBuffer(m_pointCloudMap[pointCloudSelectStr]->ssbo, GL_READ_ONLY);
				MaterialPoint* data = static_cast<MaterialPoint*>(ptr);
				std::ostringstream pointsViewStr;
				for (size_t i = 0; i < 1/*m_pointCloudMap[pointCloudSelectStr]->N*/; ++i) {
					pointsViewStr << "Material Point " << i << ":" << std::endl;
					pointsViewStr << data[i] << std::endl;
				}
				m_pointsViewStr = pointsViewStr.str();

				// set the material point
				m_mp = data[0];
				glUnmapNamedBuffer(m_pointCloudMap[pointCloudSelectStr]->ssbo);
			}
		}
		if (ImGui::CollapsingHeader("Material Points")) {
			
			ImGui::Text(m_pointsViewStr.c_str());
			if (ImGui::BeginPopupContextItem("mp item menu")) {
				if (ImGui::Button("Copy")) {
					ImGui::SetClipboardText(m_pointsViewStr.c_str());
					ImGui::CloseCurrentPopup();
				}
				if (ImGui::Button("Copy Fe")) {
					ImGui::SetClipboardText(glmToMATLAB::MatStr(m_mp.Fe).c_str());
					ImGui::CloseCurrentPopup();
				}
				if (ImGui::Button("Copy FePolar_R")) {
					ImGui::SetClipboardText(glmToMATLAB::MatStr(m_mp.FePolar_R).c_str());
					ImGui::CloseCurrentPopup();
				}
				if (ImGui::Button("Copy FePolar_S")) {
					ImGui::SetClipboardText(glmToMATLAB::MatStr(m_mp.FePolar_S).c_str());
					ImGui::CloseCurrentPopup();
				}
				if (ImGui::Button("Copy FeSVD_U")) {
					ImGui::SetClipboardText(glmToMATLAB::MatStr(m_mp.FeSVD_U).c_str());
					ImGui::CloseCurrentPopup();
				}
				if (ImGui::Button("Copy FeSVD_S")) {
					ImGui::SetClipboardText(glmToMATLAB::MatStr(m_mp.FeSVD_S).c_str());
					ImGui::CloseCurrentPopup();
				}
				if (ImGui::Button("Copy FeSVD_V")) {
					ImGui::SetClipboardText(glmToMATLAB::MatStr(m_mp.FeSVD_V).c_str());
					ImGui::CloseCurrentPopup();
				}
				ImGui::EndPopup();
			}
		}

		ImGui::End();
	}
}

void mpm::MpmEngine::HandleInput()
{
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	if (m_paused && m_rightButtonDown) {
		m_createCircleState = false;
		m_createRectState = false;
	}

	if (m_paused && m_createCircleState && m_leftButtonDown)
	{
		m_createCircleState = false;

		std::cout << "Mouse position is at (" << m_mousePos.x << ", " << m_mousePos.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_circleCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Circle shape(vec2(m_mousePos.x * GRID_SIZE_X, m_mousePos.y * GRID_SIZE_Y), m_circle_r);
		std::string circleID = "circle" + std::to_string(m_circleCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		/*color.x = (float)glm::clamp(m_color[0], 0, 255) / 255.f;
		color.y = (float)glm::clamp(m_color[1], 0, 255) / 255.f;
		color.z = (float)glm::clamp(m_color[2], 0, 255) / 255.f;*/

		real inner_rounding = m_circle_r - m_circle_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(circleID, shape, GRID_SIZE_X, GRID_SIZE_Y, inner_rounding, m_circle_rounding, m_mpParameters, m_comodel, vec2(0.0, 0.0), color);

		t2 = high_resolution_clock::now();

		

		std::cout << "Finished generating " << pointCloud->N << " points for '" << circleID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_createRectState && m_leftButtonDown)
	{
		m_createRectState = false;

		std::cout << "Mouse position is at (" << m_mousePos.x << ", " << m_mousePos.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_rectCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Rectangle shape(vec2(m_mousePos.x * GRID_SIZE_X, m_mousePos.y * GRID_SIZE_Y), m_rect_b, m_rect_h);
		std::string rectID = "rect" + std::to_string(m_rectCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		

		real inner_rounding = glm::min(m_rect_b, m_rect_h) - m_rect_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(rectID, shape, GRID_SIZE_X, GRID_SIZE_Y, inner_rounding, m_rect_rounding, m_mpParameters, m_comodel, vec2(0.0, 0.0), color);

		t2 = high_resolution_clock::now();



		std::cout << "Finished generating " << pointCloud->N << " points for '" << rectID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_createIsoTriState && m_leftButtonDown)
	{
		m_createIsoTriState = false;

		std::cout << "Mouse position is at (" << m_mousePos.x << ", " << m_mousePos.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_isoTriCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::IsoscelesTriangle shape(vec2(m_mousePos.x * GRID_SIZE_X, m_mousePos.y * GRID_SIZE_Y), m_iso_tri_b, m_iso_tri_h);
		std::string isoTriID = "isoTri" + std::to_string(m_isoTriCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

		real inner_rounding = glm::min(m_iso_tri_b, m_iso_tri_h) - m_iso_tri_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(isoTriID, shape, GRID_SIZE_X, GRID_SIZE_Y, inner_rounding, m_iso_tri_rounding, m_mpParameters, m_comodel, glm::vec2(0.f, 0.f), color);

		t2 = high_resolution_clock::now();


		std::cout << "Finished generating " << pointCloud->N << " points for '" << isoTriID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}
}


void mpm::MpmEngine::UpdateNodeData()
{
	if (0 <= m_node[0] && m_node[0] < GRID_SIZE_X && 0 <= m_node[1] && m_node[1] < GRID_SIZE_Y) {
		void *ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);
		GridNode *data = static_cast<GridNode*>(ptr);
		GridNode gn = data[m_node[0] * GRID_SIZE_X + m_node[1]];
		std::ostringstream nodeText;
		nodeText << gn << std::endl;
		m_nodeText = nodeText.str();
		glUnmapNamedBuffer(gridSSBO);
	}
}


std::shared_ptr<PointCloud> mpm::MpmEngine::GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
	const real gridDimX, const real gridDimY, 
	const real inner_rounding, const real outer_rounding,
	const MaterialParameters &parameters,
	const GLuint comodel,
	vec2 initialVelocity, glm::highp_fvec4 color)
{
	std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();

	pointCloud->color = color;
	pointCloud->parameters = parameters;
	pointCloud->mew = parameters.youngMod / (2.f + 2.f* parameters.poisson);
	pointCloud->lam = parameters.youngMod * parameters.poisson / ((1.f + parameters.poisson) * (1.f - 2.f * parameters.poisson));

	pointCloud->comodel = comodel;

	real mass = parameters.particleSpacing * parameters.particleSpacing * parameters.density;
	
	// gen points from sdf
	for (real x = 0.f; x < gridDimX; x += parameters.particleSpacing) {
		for (real y = 0.f; y < gridDimY; y += parameters.particleSpacing) {
			
			glm::vec2 p(x, y);
			real sd = shape.SdfHollow(p, inner_rounding, outer_rounding);
			if (sd < 0.f) {
				MaterialPoint mp;
				mp.x = p;
				mp.v = initialVelocity;
				mp.m = mass;
				// calculate mp.vol in a compute shader (not here)
				mp.B = mat2(0.0);
				mp.Fe = mat2(1.0);
				mp.Fp = mat2(1.0);
				mp.P = mat2(0.0); // initial Piola stress tensor is 0
				mp.FePolar_R = mat2(1.0);
				mp.FePolar_S = mat2(1.0);
				mp.FeSVD_U = mat2(1.0);
				mp.FeSVD_S = mat2(1.0);
				mp.FeSVD_V = mat2(1.0);
				
				pointCloud->points.push_back(mp);
			}
		}
	}
	pointCloud->N = pointCloud->points.size();

	if (pointCloud->N > 0) {

		// Create the SSBO for the point cloud, so it is stored on the GPU
		GLuint pointCloudSSBO;
		glCreateBuffers(1, &pointCloudSSBO);
		pointCloud->ssbo = pointCloudSSBO;
		glNamedBufferStorage(
			pointCloud->ssbo,
			sizeof(MaterialPoint) * pointCloud->points.size(),
			&(pointCloud->points.front().x.x),
			GL_MAP_READ_BIT
		);

		// Calculate volumes for the point cloud (volumes stored in SSBO on GPU)
		CalculatePointCloudVolumes(pointCloudID, pointCloud);


		m_pointCloudMap[pointCloudID] = pointCloud;
	}

	return pointCloud;
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


void mpm::MpmEngine::CreateDemo()
{
	using namespace std::chrono;

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	t1 = high_resolution_clock::now();
	m_pointCloudMap.clear();
	t2 = high_resolution_clock::now();
	std::cout << "Clearing point clouds took" << duration_cast<duration<real>>(t2 - t1).count() << " seconds.\n";


	// 1. Create point clouds
	std::cout << "Generating point cloud...\n";

	m_circleCount++;

	t1 = high_resolution_clock::now();

	
	t2 = high_resolution_clock::now();

	std::cout << "Finished generating " << m_pointCloudMap["circle1"]->N << " points for 'circle1' point cloud in " << duration_cast<duration<real>>(t2 - t1).count() << " seconds.\n";

	//1.5. Test point cloud
   //for (MaterialPoint mp : m_pointClouds[0].points) {
   //	std::cout << mp.x.x << ", " << mp.x.y << std::endl;
   //}

	//m_numPointClouds = m_pointClouds.size();
	

	//for (int i = 0; i < 100; i++) {
	//	MpmTimeStep(0.f);
	//}

	// ***** Useful test functions for reading the buffers *****
	//void *ptr = glMapNamedBuffer(pointCloudSSBO, GL_READ_ONLY);
	//MaterialPoint *data = static_cast<MaterialPoint*>(ptr);
	//for (int i = 0; i < 100; ++i) {
	//	std::cout << "Material Point " << i << ":" << std::endl;
	//	std::cout << data[i] << std::endl;
	//}
	//glUnmapNamedBuffer(pointCloudSSBO);
	/*void *ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);*/
	/*std::cout << "Reading as GridNode: \n";
	GridNode *data = static_cast<GridNode*>(ptr);
	for (int i = 0; i < GRID_SIZE_X*GRID_SIZE_Y; i++) {
		GridNode gn = data[i];
		if (gn.m != 0.f || gn.v.x != 0.f || gn.v.y != 0.f) {
			std::cout << "Grid node: [" << i / GRID_SIZE_X << ", " << i % GRID_SIZE_X << "]" << std::endl;
			std::cout << gn << std::endl;
		}
	}*/
	//std::cout << "Reading as GLfloat: \n";
	//GLfloat *test = static_cast<GLfloat*>(ptr);
	//for (int i = 0; i < GRID_SIZE_X*GRID_SIZE_Y*4; i++) {
	//	//if (test[i] != 0.f) {
	//		std::cout << "Index: " << i << std::endl;
	//		std::cout << test[i] << std::endl;
	//	//}
	//}
	/*glUnmapNamedBuffer(gridSSBO);*/
}

void mpm::MpmEngine::PrintGridData()
{
	void *ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);
	std::cout << "Reading as GridNode: \n";
	GridNode *data = static_cast<GridNode*>(ptr);
	for (int i = 0; i < GRID_SIZE_X*GRID_SIZE_Y; i++) {
		GridNode gn = data[i];
		if (gn.m != 0.0 || gn.v.x != 0.0 || gn.v.y != 0.0) {
			std::cout << "Grid node: [" << i / GRID_SIZE_X << ", " << i % GRID_SIZE_X << "]" << std::endl;
			std::cout << gn << std::endl;
		}
	}
	//std::cout << "Reading as GLfloat: \n";
	//GLfloat *test = static_cast<GLfloat*>(ptr);
	//for (int i = 0; i < GRID_SIZE_X*GRID_SIZE_Y*4; i++) {
	//	//if (test[i] != 0.f) {
	//		std::cout << "Index: " << i << std::endl;
	//		std::cout << test[i] << std::endl;
	//	//}
	//}
	glUnmapNamedBuffer(gridSSBO);
}

void mpm::MpmEngine::ChangeMaterialParameters(GLuint comodel)
{
	// save the current material parameters
	switch (m_comodel) {
	case FIXED_COROTATIONAL_ELASTICITY:
		m_fixedCorotatedParameters = m_mpParameters;
		break;
	case SIMPLE_SNOW:
		m_simpleSnowParameters = m_mpParameters;
		break;
	default:
		break;
	}
	m_comodel = comodel;
	switch (m_comodel) {
	case FIXED_COROTATIONAL_ELASTICITY:
		m_mpParameters = m_fixedCorotatedParameters;
		break;
	case SIMPLE_SNOW:
		m_mpParameters = m_simpleSnowParameters;
		break;
	default:
		break;
	}
}
