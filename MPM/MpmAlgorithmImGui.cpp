#include "MpmAlgorithmEngine.h"

void mpm::MpmAlgorithmEngine::GUI()
{
	if (m_imguiTimeIntegrator) ImGuiTimeIntegrator();
	if (m_imguiMaterialParametersEditor) ImGuiMaterialParametersEditor();
	if (m_imguiCPUMode) ImGuiCPUMode();
	if (m_imguiSparseMatrixWindow) ImGuiSparseMatrixWindow();
}

void mpm::MpmAlgorithmEngine::Menu()
{
	if (ImGui::BeginMenu("MPM")) {
		if (ImGui::MenuItem("Material Parameters Editor", "", m_imguiMaterialParametersEditor)) {
			m_imguiMaterialParametersEditor = !m_imguiMaterialParametersEditor;
		}
		if (ImGui::MenuItem("Time Integrator", "", m_imguiTimeIntegrator)) {
			m_imguiTimeIntegrator = !m_imguiTimeIntegrator;
		}
		if (ImGui::MenuItem("CPU Mode", "", m_imguiCPUMode)) {
			m_imguiCPUMode = !m_imguiCPUMode;
		}
		if (ImGui::MenuItem("Sparse Matrix Viewer", "", m_imguiSparseMatrixWindow)) {
			m_imguiSparseMatrixWindow = !m_imguiSparseMatrixWindow;
		}
		ImGui::EndMenu();
	}
}

void mpm::MpmAlgorithmEngine::ImGuiTimeIntegrator()
{
	if (ImGui::Begin("Time Integrator", &m_imguiTimeIntegrator)) {

		ImGui::Text(std::to_string(m_time).c_str());
		ImGui::Text(std::to_string(m_timeStep).c_str());
		if (ImGui::Button("Multiply dt by 2")) {
			m_dt *= 2.0;
		}
		if (ImGui::Button("Divide dt by 2")) {
			m_dt /= 2.0;
		}
		ImGui::InputReal("dt", &m_dt, 0.001, 1.0 / 60.0, "%.6f");


		ImGui::Checkbox("Realtime Rendering", &m_rt);


		ImGui::Checkbox("Implicit Time Integration (W I P)", &m_semi_implicit_CR);
		ImGui::InputReal("Implict Ratio", &m_semi_implicit_ratio);
		ImGui::InputInt("Max CR Iterations", &m_max_conj_res_iter);
		ImGui::InputReal("L2 Norm Threshold", &m_L2_norm_threshold);

		/*static size_t transferScheme = size_t(TRANSFER_SCHEME::APIC);
		m_mpmEngine->ImGuiDropDown("Transfer Scheme", transferScheme, m_transferSchemeStrVec);
		m_transferScheme = TRANSFER_SCHEME(transferScheme);*/



		ImGui::Checkbox("Paused", &m_paused);
		if (ImGui::Button("Advance") && m_paused) {
			MpmTimeStep_GLSL(m_dt);
			m_mpmEngine->UpdatePointCloudData(m_mpmEngine->m_pointCloudViewSelectStr);
			m_mpmEngine->UpdateNodeData();
		}
		if (ImGui::Button("Advance 10") && m_paused) {
			for (int i = 0; i < 10; i++) {
				MpmTimeStep_GLSL(m_dt);
			}
			m_mpmEngine->UpdatePointCloudData(m_mpmEngine->m_pointCloudViewSelectStr);
			m_mpmEngine->UpdateNodeData();
		}
		if (ImGui::Button("Advance 100") && m_paused) {
			for (int i = 0; i < 100; i++) {
				MpmTimeStep_GLSL(m_dt);
			}
			m_mpmEngine->UpdatePointCloudData(m_mpmEngine->m_pointCloudViewSelectStr);
			m_mpmEngine->UpdateNodeData();
		}
		if (ImGui::Button("Reset")) {
			MpmReset_GLSL();
		}
		ImGui::Text("MPM Algorithm Breakdown");
		if (ImGui::Button("P2G") && m_paused) {
			MpmTimeStepP2G_GLSL(m_dt);
			m_mpmEngine->UpdatePointCloudData(m_mpmEngine->m_pointCloudViewSelectStr);
			m_mpmEngine->UpdateNodeData();
		}
		if (ImGui::Button("Explicit Grid Update") && m_paused) {
			MpmTimeStepExplicitGridUpdate_GLSL(m_dt);
			m_mpmEngine->UpdatePointCloudData(m_mpmEngine->m_pointCloudViewSelectStr);
			m_mpmEngine->UpdateNodeData();
		}
		if (ImGui::Button("Semi-Implicit Grid Update") && m_paused) {
			MpmTimeStepSemiImplicitCRGridUpdate_GLSL(m_dt);
			m_mpmEngine->UpdatePointCloudData(m_mpmEngine->m_pointCloudViewSelectStr);
			m_mpmEngine->UpdateNodeData();
		}
		if (ImGui::Button("G2P") && m_paused) {
			MpmTimeStepG2P_GLSL(m_dt);
			m_mpmEngine->UpdatePointCloudData(m_mpmEngine->m_pointCloudViewSelectStr);
			m_mpmEngine->UpdateNodeData();
		}

		ImGui::Text("Conjugate residual steps (used after \"Explicit Grid Update\"");
		if (ImGui::Button("CR Init") && m_paused) {
			m_cr_step = 0;
			MpmCRInit_GLSL(m_dt);
			m_mpmEngine->UpdatePointCloudData(m_mpmEngine->m_pointCloudViewSelectStr);
			m_mpmEngine->UpdateNodeData();
		}
		bool converged = false;
		ImGui::Checkbox("Pause if not converged", &m_pause_if_not_converged);
		/*if (ImGui::Button("CR Step") && m_paused) {
			converged = MpmCRStep(m_dt);
			m_cr_step++;
			UpdateNodeData();
		}
		if (ImGui::Button("CR Step 10") && m_paused) {
			for (int i = 0; i < 10; i++) {
				converged = MpmCRStep(m_dt);
			}
			m_cr_step += 10;
			UpdateNodeData();
		}
		if (ImGui::Button("CR Step 100") && m_paused) {
			for (int i = 0; i < 100; i++) {
				converged = MpmCRStep(m_dt);
			}
			m_cr_step += 100;
			UpdateNodeData();
		}
		ImGui::Text((std::string("CR step: ") + std::to_string(m_cr_step)).c_str());*/
		if (ImGui::Button("CR End") && m_paused) {
			MpmCREnd_GLSL(m_dt);
			m_mpmEngine->UpdatePointCloudData(m_mpmEngine->m_pointCloudViewSelectStr);
			m_mpmEngine->UpdateNodeData();
		}

		//ImGui::DisplayNamedBoolColor("CR Convergence", converged, )
	}
	ImGui::End();
}





void mpm::MpmAlgorithmEngine::ImGuiMaterialParametersEditor()
{
	if (ImGui::Begin("Material Parameters Editor", &m_imguiMaterialParametersEditor)) {
		//ImGui::Color


		//ImGui::InputInt3("Color", m_color);

		static size_t energy_model = size_t(m_comodel);
		m_mpmEngine->ImGuiDropDown("Energy model", energy_model, m_energyModelsStrVec);
		if (m_comodel != ENERGY_MODEL(energy_model)) {
			ChangeEnergyModel(ENERGY_MODEL(energy_model));
		}
		switch (m_comodel) {
		case ENERGY_MODEL::LINEAR_ELASTICITY:
			ImGui::InputReal("Young's Modulus", &m_mpParameters.youngMod, 1.0, 10.0, "%.1f");
			ImGui::InputReal("Poisson's Ratio", &m_mpParameters.poisson, 0.005, 0.05, "%.3f");
			ImGui::InputReal("Density", &m_mpParameters.density, 0.01, 0.1, "%.2f");
			break;
		case ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY:
			ImGui::InputReal("Young's Modulus", &m_mpParameters.youngMod, 1.0, 10.0, "%.1f");
			ImGui::InputReal("Poisson's Ratio", &m_mpParameters.poisson, 0.005, 0.05, "%.3f");
			ImGui::InputReal("Density", &m_mpParameters.density, 0.01, 0.1, "%.2f");
			break;
		case ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY:
			ImGui::InputReal("Young's Modulus", &m_mpParameters.youngMod, 1.0, 10.0, "%.1f");
			ImGui::InputReal("Poisson's Ratio", &m_mpParameters.poisson, 0.005, 0.05, "%.3f");
			ImGui::InputReal("Density", &m_mpParameters.density, 0.01, 0.1, "%.2f");
			break;
		case ENERGY_MODEL::SIMPLE_SNOW:
			ImGui::InputReal("Young's Modulus", &m_mpParameters.youngMod, 1.0, 10.0, "%.1f");
			ImGui::InputReal("Poisson's Ratio", &m_mpParameters.poisson, 0.005, 0.05, "%.3f");
			ImGui::InputReal("Density", &m_mpParameters.density, 0.01, 0.1, "%.2f");
			ImGui::InputReal("Critical Compression", &m_mpParameters.crit_c, 0.001, 0.01, "%.4f");
			ImGui::InputReal("Critical Stretch", &m_mpParameters.crit_s, 0.001, 0.01, "%.4f");
			ImGui::InputReal("Hardening", &m_mpParameters.hardening, 0.001, 0.01, "%.4f");
			break;
		default:
			break;
		}
	}
	ImGui::End();
}
void mpm::MpmAlgorithmEngine::ImGuiCPUMode()
{
	if (ImGui::Begin("CPU Mode", &m_imguiCPUMode)) {
		ImGui::Text("MLS-MPM and Fixed Corotational Elasticity are implemented");
		static bool cpu_mode = true;
		ImGui::Checkbox("CPU Mode", &cpu_mode);
		if (cpu_mode) {
			m_algo_code = MPM_ALGORITHM_CODE::CPP;
		}
		else {
			m_algo_code = MPM_ALGORITHM_CODE::GLSL;
		}

		//const static std::vector<MPM_ALGO> mpmAlgoList = std::vector<MPM_ALGO
		static size_t transferScheme = size_t(TRANSFER_SCHEME::APIC);
		m_mpmEngine->ImGuiDropDown("Transfer Scheme", transferScheme, m_transferSchemeStrVec);
		m_transferScheme = TRANSFER_SCHEME(transferScheme);

		static size_t mpmAlgo = size_t(m_mpm_algo);
		m_mpmEngine->ImGuiDropDown("MPM Algorithm", mpmAlgo, m_mpmAlgoStrVec);
		m_mpm_algo = MPM_ALGO(mpmAlgo);

		static size_t basisFn = size_t(m_basisFunction);
		m_mpmEngine->ImGuiDropDown("MPM Basis Function", basisFn, m_basisFunctionStrVec);
		m_basisFunction = Basis::BasisType(basisFn);

		ImGui::InputInt("CPU Mode Chunk Size X", &m_cppChunkX, 1, 4);
		ImGui::InputInt("CPU Mode Chunk Size Y", &m_cppChunkY, 1, 4);
		m_cppChunkX = glm::max(glm::min(32, m_cppChunkX), 0);
		m_cppChunkY = glm::max(glm::min(32, m_cppChunkY), 0);

		if (ImGui::Button("32 x 32 grid")) {
			m_mpmEngine->m_chunks_x = 1;
			m_mpmEngine->m_chunks_y = 1;
			m_cppChunkX = 32;
			m_cppChunkY = 32;
		}
		if (ImGui::Button("20 x 20 grid")) {
			m_mpmEngine->m_chunks_x = 1;
			m_mpmEngine->m_chunks_y = 1;
			m_cppChunkX = 20;
			m_cppChunkY = 20;
		}
		if (ImGui::Button("Small circle")) {
			m_mpmGeometryEngine->SmallCircle();
		}

		ImGui::Text(std::to_string(m_time).c_str());
		ImGui::Text(std::to_string(m_timeStep).c_str());
		if (ImGui::Button("Multiply dt by 2")) {
			m_dt *= 2.0;
		}
		if (ImGui::Button("Divide dt by 2")) {
			m_dt /= 2.0;
		}
		ImGui::InputReal("dt", &m_dt, 0.001, 1.0 / 60.0, "%.6f");

		ImGui::Checkbox("Realtime rendering", &m_rt);

		ImGui::Checkbox("Paused", &m_paused);

		ImGui::Checkbox("USL/USF", &m_USL);

		ImGui::Checkbox("Semi Implict Time Integration", &m_semiImplicitCPP);
		ImGui::InputDouble("Implicit Ratio (beta)", &m_beta, 0.05, 0.1);
		m_beta = glm::max(glm::min(1.0, m_beta), 0.0);

		if (ImGui::Button("Advance") && m_paused) {
			MpmTimeStep_MLS(m_dt);
			m_mpmEngine->MapCPUPointCloudsToGPU();
			m_mpmEngine->MapCPUGridToGPU();
		}

		if (ImGui::Button("Reset")) {
			MpmReset_MLS();
		}


	}
	ImGui::End();
}

void mpm::MpmAlgorithmEngine::ImGuiSparseMatrixWindow()
{
	if (ImGui::Begin("Sparse Matrix Viewer", &m_imguiSparseMatrixWindow)) {

		ImGui::Checkbox("Visualize Semi-Implicit Euler Matrix", &m_visSemiImplicitEulerMatrix);

		ImGui::Image(
			(void*)(intptr_t)m_sparseMatrixWindow->texture,
			ImVec2((float)m_sparseMatrixWindow->screen_dimensions.x, (float)m_sparseMatrixWindow->screen_dimensions.y),
			ImVec2(0, 1),
			ImVec2(1, 0),
			ImVec4(1, 1, 1, 1),
			ImVec4(1, 1, 1, 1)
		);
	}
	ImGui::End();
}