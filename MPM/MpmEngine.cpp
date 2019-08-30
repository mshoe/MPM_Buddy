#include "MpmEngine.h"
#include "imgui/imgui.h"

float BSpline(float x) {
	return (x < 0.5f) ? glm::step(0.0f, x)*(0.75f - x * x) :
		glm::step(x, 1.5f)*0.5f*(1.5f - abs(x))*(1.5f - abs(x));
}

float BSplineSlope(float x) {
	return (x < 0.5f) ? glm::step(0.0f, x)*(-2.f * x) :
		glm::step(x, 1.5f)*(1.5f - abs(x))*x / abs(x);
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

	return true;
}

bool mpm::MpmEngine::CleanupComputeShaderPipeline()
{
	m_pointCloudMap.clear();
	glDeleteBuffers(1, &gridSSBO);
	glDeleteVertexArrays(1, &VisualizeVAO);

	return false;
}

void mpm::MpmEngine::MpmTimeStep(float dt)
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
		m_p2gScatter->SetFloat("dt", dt);
		int g2p_workgroups = int(glm::ceil(float(pointCloudPair.second->N) / float(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(g2p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gUpdate->Use();
	m_gUpdate->SetFloat("dt", dt);
	m_gUpdate->SetVec("globalForce", m_globalForce);
	m_gUpdate->SetVec("mousePos", m_mousePos);
	m_gUpdate->SetFloat("drag", m_drag);
	if (m_rightButtonDown)
		m_gUpdate->SetFloat("mousePower", m_mousePower);
	else
		m_gUpdate->SetFloat("mousePower", 0.f);
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	if (m_implicit) {
		MpmImplictTimeCR(dt);
	}

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		m_g2pGather->Use();
		m_g2pGather->SetFloat("dt", dt);
		m_g2pGather->SetFloat("lam", pointCloudPair.second->lam);
		m_g2pGather->SetFloat("mew", pointCloudPair.second->mew);
		m_g2pGather->SetFloat("crit_c", pointCloudPair.second->crit_c);
		m_g2pGather->SetFloat("crit_s", pointCloudPair.second->crit_s);
		m_g2pGather->SetFloat("hardening", pointCloudPair.second->hardening);
		m_g2pGather->SetuInt("comodel", pointCloudPair.second->comodel);
		int g2p_workgroups = int(glm::ceil(float(pointCloudPair.second->N) / float(G2P_WORKGROUP_SIZE)));
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

void mpm::MpmEngine::MpmImplictTimeCR(float dt)
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
		m_p2g2pDeltaForce->SetFloat("lam", pointCloudPair.second->lam);
		m_p2g2pDeltaForce->SetFloat("mew", pointCloudPair.second->mew);
		int p_workgroups = int(glm::ceil(float(pointCloudPair.second->N) / float(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}

#ifdef MPM_CR_DEBUG
	PrintGridData();
#endif

	// IMPLICIT INIT PART 2
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsInitPart2->Use();
	m_gConjugateResidualsInitPart2->SetFloat("dt", dt);
	m_gConjugateResidualsInitPart2->SetFloat("IMPLICIT_RATIO", m_implicit_ratio);
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
		m_p2g2pDeltaForce->SetFloat("lam", pointCloudPair.second->lam);
		m_p2g2pDeltaForce->SetFloat("mew", pointCloudPair.second->mew);
		int p_workgroups = int(glm::ceil(float(pointCloudPair.second->N) / float(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}

#ifdef MPM_CR_DEBUG
	PrintGridData();
#endif

	// IMPLICIT INIT PART 3
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gConjugateResidualsInitPart3->Use();
	m_gConjugateResidualsInitPart3->SetFloat("dt", dt);
	m_gConjugateResidualsInitPart3->SetFloat("IMPLICIT_RATIO", m_implicit_ratio);
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
			m_p2g2pDeltaForce->SetFloat("lam", pointCloudPair.second->lam);
			m_p2g2pDeltaForce->SetFloat("mew", pointCloudPair.second->mew);
			int p_workgroups = int(glm::ceil(float(pointCloudPair.second->N) / float(G2P_WORKGROUP_SIZE)));
			glDispatchCompute(p_workgroups, 1, 1);
			glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		}

#ifdef MPM_CR_DEBUG
		PrintGridData();
#endif

		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		m_gConjugateResidualsStepPart2->Use();
		m_gConjugateResidualsStepPart2->SetFloat("dt", dt);
		m_gConjugateResidualsStepPart2->SetFloat("IMPLICIT_RATIO", m_implicit_ratio);
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
			float curr_dt = 0.f;
			float rt_dt = 1.f / 60.f;
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

		
		ImGui::InputFloat("drag", &m_drag, 0.0001f, 0.01f, "%.4f");

		if (ImGui::Button("Set global force")) {
			m_globalForce.x = m_globalForceArray[0];
			m_globalForce.y = m_globalForceArray[1];
		}
		ImGui::InputFloat2("", m_globalForceArray, "%.3f");

		ImGui::InputFloat("Mouse power", &m_mousePower);

		ImGui::Text(std::to_string(m_time).c_str());
		ImGui::Text(std::to_string(m_timeStep).c_str());
		ImGui::InputFloat("dt", &m_dt, 0.001f, 1.f/60.f, "%.6f");

		ImGui::Checkbox("Realtime Rendering", &m_rt);

		/*ImGui::InputFloat2("circle init x", m_circle_x, "%.3f");
		ImGui::InputFloat2("circle init v", m_circle_v, "%.3f");
		ImGui::InputFloat2("donut init x", m_donut_x, "%.3f");
		ImGui::InputFloat2("donut init v", m_donut_v, "%.3f");*/
		
		
		ImGui::Checkbox("Node Selection Graphics", &m_nodeGraphicsActive);
		if (ImGui::Button("Get node data") && m_paused) {
			UpdateNodeData();
		}
		ImGui::InputInt2("Grid Node:", m_node);
		ImGui::Text(m_nodeText.c_str());
		
		ImGui::Checkbox("Implicit Time Integration", &m_implicit);
		ImGui::InputFloat("Implict Ratio", &m_implicit_ratio);

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
			m_time = 0.f;
		}

		ImGui::End();

		ImGui::Begin("Geometry Editor");

		//ImGui::Color
		ImGui::ColorEdit4("Color", m_color);
		//ImGui::InputInt3("Color", m_color);

		ImGui::InputFloat("Young's Modulus", &m_youngMod, 1.f, 10.f, "%.1f");
		ImGui::InputFloat("Poisson's Ratio", &m_poisson, 0.005f, 0.05f, "%.3f");
		ImGui::InputFloat("Point Spacing", &m_particleSpacing, 0.01f, 0.1f, "%.2f");
		ImGui::InputFloat("Density", &m_density, 0.01f, 0.1f, "%.2f");
		ImGui::InputFloat("Critical Compression", &m_crit_c, 0.001f, 0.01f, "%.4f");
		ImGui::InputFloat("Critical Stretch", &m_crit_s, 0.001f, 0.01f, "%.4f");
		ImGui::InputFloat("Hardening", &m_hardening, 0.001f, 0.01f, "%.4f");

		if (ImGui::Button("Fixed Corotated Elasticity")) {
			m_comodel = 1;
		}
		if (ImGui::Button("Stovakhim Snow (2013)")) {
			m_comodel = 2;
		}
		std::string comodelStr = "constitutive model: " + std::to_string(m_comodel);
		ImGui::Text(comodelStr.c_str());



		ImGui::InputFloat("Circle Radius", &m_circle_r, 0.1f, 1.f, "%.1f");
		ImGui::InputFloat("Circle Inner Radius", &m_circle_inner_radius, 0.1f, 1.f, "%.1f");
		ImGui::InputFloat("Circle Rounding", &m_circle_rounding, 0.1f, 1.f, "%.1f");
		if (ImGui::Button("Create Solid Circle") && m_paused) {
			m_createCircleState = true;
		}


		ImGui::InputFloat("Rectangle Base Length", &m_rect_b, 0.1f, 1.f, "%.1f");
		ImGui::InputFloat("Rectangle Height Length", &m_rect_h, 0.1f, 1.f, "%.1f");
		ImGui::InputFloat("Rectangle Inner Radius", &m_rect_inner_radius, 0.1f, 1.f, "%.1f");
		ImGui::InputFloat("Rectangle Rounding", &m_rect_rounding, 0.1f, 1.f, "%.1f");
		if (ImGui::Button("Create Solid Rectangle") && m_paused) {
			m_createRectState = true;
		}

		ImGui::InputFloat("Isosceles Triangle Base Length", &m_iso_tri_b, 0.1f, 1.f, "%.1f");
		ImGui::InputFloat("Isosceles Height Length", &m_iso_tri_h, 0.1f, 1.f, "%.1f");
		ImGui::InputFloat("Isosceles Triangle Inner Radius", &m_iso_tri_inner_radius, 0.1f, 1.f, "%.1f");
		ImGui::InputFloat("Isosceles Triangle Rounding", &m_iso_tri_rounding, 0.1f, 1.f, "%.1f");
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
				for (size_t i = 0; i < m_pointCloudMap[pointCloudSelectStr]->N; ++i) {
					pointsViewStr << "Material Point " << i << ":" << std::endl;
					pointsViewStr << data[i] << std::endl;
				}
				m_pointsViewStr = pointsViewStr.str();
				glUnmapNamedBuffer(m_pointCloudMap[pointCloudSelectStr]->ssbo);
			}
		}
		if (ImGui::CollapsingHeader("Material Points")) {
			ImGui::Text(m_pointsViewStr.c_str());
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
		sdf::Circle shape(glm::vec2(m_mousePos.x * GRID_SIZE_X, m_mousePos.y * GRID_SIZE_Y), m_circle_r);
		std::string circleID = "circle" + std::to_string(m_circleCount);

		glm::vec4 color = glm::vec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		/*color.x = (float)glm::clamp(m_color[0], 0, 255) / 255.f;
		color.y = (float)glm::clamp(m_color[1], 0, 255) / 255.f;
		color.z = (float)glm::clamp(m_color[2], 0, 255) / 255.f;*/

		float inner_rounding = m_circle_r - m_circle_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(circleID, shape, GRID_SIZE_X, GRID_SIZE_Y, m_particleSpacing, m_density, inner_rounding, m_circle_rounding, m_youngMod, m_poisson, m_crit_c, m_crit_s, m_hardening, m_comodel, glm::vec2(0.f, 0.f), color);

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
		sdf::Rectangle shape(glm::vec2(m_mousePos.x * GRID_SIZE_X, m_mousePos.y * GRID_SIZE_Y), m_rect_b, m_rect_h);
		std::string rectID = "rect" + std::to_string(m_rectCount);

		glm::vec4 color = glm::vec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		

		float inner_rounding = glm::min(m_rect_b, m_rect_h) - m_rect_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(rectID, shape, GRID_SIZE_X, GRID_SIZE_Y, m_particleSpacing, m_density, inner_rounding, m_rect_rounding, m_youngMod, m_poisson, m_crit_c, m_crit_s, m_hardening, m_comodel, glm::vec2(0.f, 0.f), color);

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
		sdf::IsoscelesTriangle shape(glm::vec2(m_mousePos.x * GRID_SIZE_X, m_mousePos.y * GRID_SIZE_Y), m_iso_tri_b, m_iso_tri_h);
		std::string isoTriID = "isoTri" + std::to_string(m_isoTriCount);

		glm::vec4 color = glm::vec4(m_color[0], m_color[1], m_color[2], m_color[3]);

		float inner_rounding = glm::min(m_iso_tri_b, m_iso_tri_h) - m_iso_tri_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(isoTriID, shape, GRID_SIZE_X, GRID_SIZE_Y, m_particleSpacing, m_density, inner_rounding, m_iso_tri_rounding, m_youngMod, m_poisson, m_crit_c, m_crit_s, m_hardening, m_comodel, glm::vec2(0.f, 0.f), color);

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
	const float gridDimX, const float gridDimY, 
	const float particleSpacing, const float density, 
	const float inner_rounding, const float outer_rounding,
	const float youngMod, const float poisson,
	const float crit_c, const float crit_s, const float hardening,
	const GLuint comodel,
	glm::vec2 initialVelocity, glm::vec4 color)
{
	std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();

	pointCloud->color = color;

	pointCloud->mew = youngMod / (2.f + 2.f*poisson);
	pointCloud->lam = youngMod * poisson / ((1.f + poisson) * (1.f - 2.f * poisson));
	pointCloud->crit_c = crit_c;
	pointCloud->crit_s = crit_s;
	pointCloud->hardening = hardening;
	pointCloud->comodel = comodel;

	float mass = particleSpacing * particleSpacing * density;
	
	// gen points from sdf
	for (float x = 0.f; x < gridDimX; x += particleSpacing) {
		for (float y = 0.f; y < gridDimY; y += particleSpacing) {
			
			glm::vec2 p(x, y);
			float sd = shape.SdfHollow(p, inner_rounding, outer_rounding);
			if (sd < 0.f) {
				MaterialPoint mp;
				mp.x = p;
				mp.v = initialVelocity;
				mp.m = mass;
				// calculate mp.vol in a compute shader (not here)
				mp.B = glm::mat2(0.f);
				mp.Fe = glm::mat2(1.f);
				mp.Fp = glm::mat2(1.f);
				mp.P = glm::mat2(0.f); // initial Piola stress tensor is 0
				mp.FeSVD_U = glm::mat2(1.f);
				mp.FeSVD_S = glm::mat2(1.f);
				mp.FeSVD_V = glm::mat2(1.f);
				
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
	int g2p_workgroups = int(glm::ceil(float(pointCloud->N) / float(G2P_WORKGROUP_SIZE)));
	glDispatchCompute(g2p_workgroups, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	t2 = high_resolution_clock::now();
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);

	std::cout << "Finished calculating initial volumes for '" << pointCloudID << "' in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
}


void mpm::MpmEngine::CreateDemo()
{
	using namespace std::chrono;

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	t1 = high_resolution_clock::now();
	m_pointCloudMap.clear();
	t2 = high_resolution_clock::now();
	std::cout << "Clearing point clouds took" << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";


	// 1. Create point clouds
	std::cout << "Generating point cloud...\n";

	m_circleCount++;

	t1 = high_resolution_clock::now();

	
	t2 = high_resolution_clock::now();

	std::cout << "Finished generating " << m_pointCloudMap["circle1"]->N << " points for 'circle1' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";

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
		if (gn.m != 0.f || gn.v.x != 0.f || gn.v.y != 0.f) {
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
