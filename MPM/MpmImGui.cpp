#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"

void mpm::MpmEngine::RenderGUI()
{
	if (m_renderGUI) {

		RenderTimeIntegrator();
		RenderForceController();
		RenderGeometryEditor();
		RenderGridNodeViewer();
		RenderMaterialPointViewer();


	}
}

void mpm::MpmEngine::RenderTimeIntegrator()
{
	ImGui::Begin("Time Integrator", &m_renderGUI);

	ImGui::Text(std::to_string(m_time).c_str());
	ImGui::Text(std::to_string(m_timeStep).c_str());
	ImGui::InputReal("dt", &m_dt, 0.001, 1.0 / 60.0, "%.6f");

	ImGui::Checkbox("Realtime Rendering", &m_rt);


	ImGui::Checkbox("Implicit Time Integration", &m_implicit);
	ImGui::InputReal("Implict Ratio", &m_implicit_ratio);
	ImGui::InputInt("Max CR Iterations", &m_max_conj_res_iter);

	if (ImGui::Button("Pause")) {
		m_paused = !m_paused;
	}
	if (ImGui::Button("Advance") && m_paused) {
		MpmTimeStep(m_dt);
		UpdateNodeData();
	}
	if (ImGui::Button("Advance 10") && m_paused) {
		for (int i = 0; i < 10; i++) {
			MpmTimeStep(m_dt);
		}
		UpdateNodeData();
	}
	if (ImGui::Button("Advance 100") && m_paused) {
		for (int i = 0; i < 100; i++) {
			MpmTimeStep(m_dt);
		}
		UpdateNodeData();
	}
	if (ImGui::Button("Restart")) {
		m_pointCloudMap.clear();
		m_timeStep = 0;
		m_time = 0.0;
	}
	ImGui::Text("MPM Algorithm Breakdown");
	if (ImGui::Button("P2G") && m_paused) {
		MpmTimeStepP2G(m_dt);
		UpdateNodeData();
	}
	if (ImGui::Button("Explicit Grid Update") && m_paused) {
		MpmTimeStepExplicitGridUpdate(m_dt);
		UpdateNodeData();
	}
	if (ImGui::Button("Semi-Implicit Grid Update") && m_paused) {
		MpmTimeStepSemiImplicitGridUpdate(m_dt);
		UpdateNodeData();
	}
	if (ImGui::Button("G2P") && m_paused) {
		MpmTimeStepG2P(m_dt);
		UpdateNodeData();
	}

	ImGui::Text("Conjugate residual steps (used after \"Explicit Grid Update\"");
	if (ImGui::Button("CR Init") && m_paused) {
		m_cr_step = 0;
		MpmCRInit(m_dt);
		UpdateNodeData();
	}
	bool converged = false;

	if (ImGui::Button("CR Step") && m_paused) {
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
	ImGui::Text((std::string("CR step: ") + std::to_string(m_cr_step)).c_str());
	if (ImGui::Button("CR End") && m_paused) {
		MpmCREnd(m_dt);
		UpdateNodeData();
	}
	//ImGui::DisplayNamedBoolColor("CR Convergence", converged, )





	ImGui::End();
}

void mpm::MpmEngine::RenderForceController()
{
	ImGui::Begin("MPM Force Controller", &m_renderGUI);


	ImGui::InputReal("drag", &m_drag, 0.0001, 0.01, "%.4f");

	ImGui::InputReal("Global Force x", &m_globalForce.x, 0.1, 1.0, "%.16f");
	ImGui::InputReal("Global Force y", &m_globalForce.y, 0.1, 1.0, "%.16f");

	ImGui::InputReal("Mouse power", &m_mousePower);

	ImGui::End();
}

void mpm::MpmEngine::RenderGeometryEditor()
{
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
}

void mpm::MpmEngine::RenderGridNodeViewer()
{
	ImGui::Begin("Grid Node Viewer");

	ImGui::Checkbox("Node Selection Graphics", &m_nodeGraphicsActive);
	if (ImGui::Button("Select Node")) {
		m_selectNodeState = true;
	}
	ImGui::InputInt2("Grid Node:", m_node);
	if (ImGui::Button("Get node data") && m_paused) {
		UpdateNodeData();
	}
	if (ImGui::CollapsingHeader("Grid Node Data")) {
		glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
		glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);
		ImGui::DisplayNamedGlmRealColor("m", m_gn.m, max_color);
		ImGui::DisplayNamedGlmVecMixColor("v", m_gn.v, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("mv", m_gn.momentum, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("f", m_gn.force, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("df", m_gn.deltaForce, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("rk", m_gn.rk, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("xk", m_gn.xk, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("pk", m_gn.pk, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("Ark", m_gn.Ark, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("Apk", m_gn.Apk, min_color, max_color);
		ImGui::DisplayNamedGlmRealColor("rkArk", m_gn.rkArk, max_color);
		real rk_valueSq = glm::dot(m_gn.rk, m_gn.rk);
		ImGui::DisplayNamedGlmRealColor("|rk|^2", rk_valueSq, max_color);
		ImGui::DisplayNamedBoolColor("|rk|^2 < 0.0001", m_gn.converged, max_color, min_color);
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderMaterialPointViewer()
{
	ImGui::Begin("Material Point Viewer");

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
			m_mp = data[0];
			glUnmapNamedBuffer(m_pointCloudMap[pointCloudSelectStr]->ssbo);
		}
	}
	ImGui::InputReal("Max energy clamp (for coloring)", &m_maxEnergyClamp, 1.0, 1000.0, "%.1f");
	ImGui::InputReal("Min energy clamp (for coloring)", &m_minEnergyClamp, 1.0, 1000.0, "%.1f");
	ImGui::Checkbox("Visualize Energy", &m_visualizeEnergy);

	if (ImGui::Button("Set Reference Configuration")) {
		SetReferenceConfig(pointCloudSelectStr);
	}

	if (ImGui::CollapsingHeader("Material Point")) {
		glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
		glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);

		ImGui::DisplayNamedGlmVecMixColor("x", m_mp.x, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("v", m_mp.v, min_color, max_color);
		ImGui::DisplayNamedGlmRealColor("m", m_mp.m, max_color);
		ImGui::DisplayNamedGlmRealColor("vol", m_mp.vol, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("B", m_mp.B, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("Fe", m_mp.Fe, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("Fp", m_mp.Fp, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("P", m_mp.P, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("FePolar_R", m_mp.FePolar_R, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("FePolar_S", m_mp.FePolar_S, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("FeSVD_U", m_mp.FeSVD_U, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("FeSVD_S", m_mp.FeSVD_S, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("FeSVD_V", m_mp.FeSVD_V, min_color, max_color);
		ImGui::DisplayNamedGlmRealColor("energy", m_mp.energy, max_color);
	}

	ImGui::End();
}