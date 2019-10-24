#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"

void mpm::MpmEngine::RenderGUI()
{
	
	if (m_renderGUI) {

		RenderWindowManager();
		if (m_renderTimeIntegrator) RenderTimeIntegrator();
		if (m_renderForceController) RenderForceController();
		if (m_renderGeometryEditor) RenderGeometryEditor();
		if (m_renderMaterialParametersEditor) RenderMaterialParametersEditor();
		if (m_renderGridNodeViewer) RenderGridNodeViewer();
		if (m_renderMaterialPointViewer) RenderMaterialPointViewer();
		if (m_renderZoomWindow) RenderZoomWindow();
		
	}
}

void mpm::MpmEngine::RenderWindowManager()
{
	ImGui::Begin("Window Manager");

	ImGui::Checkbox("Time Integrator", &m_renderTimeIntegrator);
	ImGui::Checkbox("Force Controller", &m_renderForceController);
	ImGui::Checkbox("Geometry Editor", &m_renderGeometryEditor);
	ImGui::Checkbox("Material Parameters Editor", &m_renderMaterialParametersEditor);
	ImGui::Checkbox("Grid Node Viewer", &m_renderGridNodeViewer);
	ImGui::Checkbox("Material Point Viewer", &m_renderMaterialPointViewer);
	ImGui::Checkbox("Zoom Window", &m_renderZoomWindow);

	static bool renderImguiDemo = false;
	ImGui::Checkbox("ImGui Demo", &renderImguiDemo);
	if (renderImguiDemo) { ImGui::ShowDemoWindow(); }

	ImGui::End();
}

void mpm::MpmEngine::RenderTimeIntegrator()
{
	ImGui::Begin("Time Integrator");

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

	static int transferScheme = int(TRANSFER_SCHEME::APIC);
	ImGui::InputInt("Transfer scheme", &transferScheme);
	transferScheme = glm::max(glm::min(transferScheme, int(TRANSFER_SCHEME::APIC)), int(TRANSFER_SCHEME::PIC));
	m_transferScheme = TRANSFER_SCHEME(transferScheme);



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
	if (ImGui::Button("Reset")) {
		MpmReset();
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
		MpmTimeStepSemiImplicitCRGridUpdate(m_dt);
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

	if (ImGui::Button("Set Current Point Cloud Deformation Gradients")) {
		SetDeformationGradients(std::string(m_pointCloudSelect.data()), m_setFe, m_setFp);
	}
	if (ImGui::Button("Set All Deformation Gradients")) {
		for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
			SetDeformationGradients(pointCloudPair.first, m_setFe, m_setFp);
		}
	}

	mat2 tempFe = m_setFe;

	ImGui::Text("Fe:");
	ImGui::InputReal("Fe[0][0]: ", &tempFe[0][0], 0.1, 1.0, "%.6f");
	ImGui::InputReal("Fe[0][1]: ", &tempFe[0][1], 0.1, 1.0, "%.6f");
	ImGui::InputReal("Fe[1][0]: ", &tempFe[1][0], 0.1, 1.0, "%.6f");
	ImGui::InputReal("Fe[1][1]: ", &tempFe[1][1], 0.1, 1.0, "%.6f");
	// Make sure the Fe is non-singular
	if (glm::determinant(tempFe) != 0.0) {
		m_setFe = tempFe;
	}
	ImGui::DisplayNamedGlmMatrixMixColor("Fe: ", m_setFe, glm::highp_fvec4(1.0f, 0.0f, 0.0f, 1.0f), glm::highp_fvec4(0.0f, 1.0f, 0.0f, 1.0f));

	
	
	if (ImGui::Button("Clear multFeVector")) {
		m_multFeVector.clear();
	}
	if (ImGui::Button("Add multiplicator deformation gradient")) {
		m_multFeVector.push_back(mat2(1.0));
	}

	static int multFselect = 0;

	ImGui::InputInt("multFe selection", &multFselect);

	if (multFselect < m_multFeVector.size() && multFselect >= 0) {
		ImGui::InputReal("multFe[0][0]: ", &m_multFeVector[multFselect][0][0], 0.1, 1.0, "%.6f");
		ImGui::InputReal("multFe[0][1]: ", &m_multFeVector[multFselect][0][1], 0.1, 1.0, "%.6f");
		ImGui::InputReal("multFe[1][0]: ", &m_multFeVector[multFselect][1][0], 0.1, 1.0, "%.6f");
		ImGui::InputReal("multFe[1][1]: ", &m_multFeVector[multFselect][1][1], 0.1, 1.0, "%.6f");
	}

	for (size_t i = 0; i < m_multFeVector.size(); i++) {
		std::string multFeStr = "Multiply all Deformation Gradients by multFe(" + std::to_string(i) + "):";
		if (ImGui::Button(multFeStr.c_str())) {
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
				MultiplyDeformationGradients(pointCloudPair.first, m_multFeVector[i], m_multFp);
			}
		}
		
		ImGui::DisplayNamedGlmMatrixMixColor("multFe: ", m_multFeVector[i], glm::highp_fvec4(1.0f, 0.0f, 0.0f, 1.0f), glm::highp_fvec4(0.0f, 1.0f, 0.0f, 1.0f));
	}

	if (ImGui::Button("Set All Point Cloud Parameters")) {
		for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
			pointCloudPair.second->parameters = m_mpParameters;
			pointCloudPair.second->mew = m_mpParameters.youngMod / (2.f + 2.f * m_mpParameters.poisson);
			pointCloudPair.second->lam = m_mpParameters.youngMod * m_mpParameters.poisson / ((1.f + m_mpParameters.poisson) * (1.f - 2.f * m_mpParameters.poisson));
		}
	}

	ImGui::InputReal("lam", &m_lam);
	ImGui::InputReal("mew", &m_mew);
	if (ImGui::Button("Set All Point Cloud mew and lam")) {
		for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
			pointCloudPair.second->mew = m_mew;
			pointCloudPair.second->lam = m_lam;
		}
	}

	ImGui::End();
}

void mpm::MpmEngine::RenderGeometryEditor()
{
	ImGui::Begin("Geometry Editor");

	

	ImGui::Checkbox("Fixed point cloud", &m_fixedPointCloud);
	ImGui::Checkbox("Inverted SDF (DANGER)", &m_invertedSdf);

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

	//ImGui::InputReal("Line m", &m_line_m);
	//ImGui::InputReal("Line b", &m_line_b);
	//ImGui::InputReal("Line2 m", &m_line2_m);
	//ImGui::InputReal("Line2 b", &m_line2_b);
	//if (ImGui::Button("Create below line y = mx + b")) {
	//	GenPointCloudLineDivider();
	//}

	ImGui::Text("");
	ImGui::Checkbox("Render polygon", &m_renderPolygon);
	if (ImGui::Button("Add Polygon Vertex")) {
		m_addPolygonVertexState = true;
	}
	if (ImGui::Button("Clear Polygon")) {
		m_polygon->vertices.clear();
	}
	if (ImGui::Button("Fill polygon with MPs")) {
		GenPointCloudPolygon();
	}
	if (ImGui::Button("Select grid nodes")) {
		
	}
	ImGui::DisplayNamedGlmRealColor("Number of vertices", m_polygon->vertices.size(), glm::highp_fvec4(1.0));
	ImGui::Text("Polygon vertices:");
	for (int i = 0; i < m_polygon->vertices.size(); i++) {
		ImGui::DisplayGlmVec(m_polygon->vertices[i]);
	}

	ImGui::Text("");
	ImGui::Checkbox("Render piecewise line", &m_renderPWLine);
	if (ImGui::Button("Add line Vertex")) {
		m_addPWLineVertexState = true;
	}
	if (ImGui::Button("Clear Line")) {
		m_pwLine->vertices.clear();
	}
	if (ImGui::Button("Create PW Line from SDF")) {
		GenPointCloudPWLine();
	}
	ImGui::InputReal("pwLineRounding", &m_pwLineRounding, 0.1, 1.0, "%.6f");
	ImGui::DisplayNamedGlmRealColor("Number of vertices", m_pwLine->vertices.size(), glm::highp_fvec4(1.0));
	ImGui::Text("PW Line vertices:");
	for (int i = 0; i < m_pwLine->vertices.size(); i++) {
		ImGui::DisplayGlmVec(m_pwLine->vertices[i]);
	}


	ImGui::End();
}

void mpm::MpmEngine::RenderMaterialParametersEditor()
{
	ImGui::Begin("Material Parameters Editor");
	//ImGui::Color
	ImGui::ColorEdit4("Color", m_color);
	ImGui::InputReal("Initial Velocity X", &m_initVelocity.x, 0.1, 1.0, "%.1f");
	ImGui::InputReal("Initial Velocity Y", &m_initVelocity.y, 0.1, 1.0, "%.1f");

	//ImGui::InputInt3("Color", m_color);

	switch (m_comodel) {
	case NEO_HOOKEAN_ELASTICITY:
		ImGui::InputReal("Young's Modulus", &m_mpParameters.youngMod, 1.0, 10.0, "%.1f");
		ImGui::InputReal("Poisson's Ratio", &m_mpParameters.poisson, 0.005, 0.05, "%.3f");
		ImGui::InputReal("Point Spacing", &m_mpParameters.particleSpacing, 0.01, 0.1, "%.2f");
		ImGui::InputReal("Density", &m_mpParameters.density, 0.01, 0.1, "%.2f");
		break;
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

	if (ImGui::Button("Neo-Hookean Elasticity")) {
		ChangeMaterialParameters(NEO_HOOKEAN_ELASTICITY);
	}
	if (ImGui::Button("Fixed Corotated Elasticity")) {
		ChangeMaterialParameters(FIXED_COROTATIONAL_ELASTICITY);
	}
	if (ImGui::Button("Stovakhim Snow (2013)")) {
		ChangeMaterialParameters(SIMPLE_SNOW);
	}
	std::string comodelStr = "constitutive model: " + std::to_string(m_comodel);
	ImGui::Text(comodelStr.c_str());
	ImGui::End();
}

void mpm::MpmEngine::RenderGridNodeViewer()
{
	ImGui::Begin("Grid Node Viewer");

	

	std::string chunkWidthStr = "Chunk width: " + std::to_string(CHUNK_WIDTH);
	ImGui::Text(chunkWidthStr.c_str());

	ImGui::InputInt("# chunks (x)", &m_chunks_x, 1, 1);
	m_chunks_x = glm::clamp(m_chunks_x, 1, 4);
	ImGui::InputInt("# chunks (y)", &m_chunks_y, 1, 1);
	m_chunks_y = glm::clamp(m_chunks_y, 1, 4);

	ImGui::Checkbox("Collective node selection graphics", &m_collectiveNodeSelectionGraphics);
	if (ImGui::Button("Select nodes in polygon")) {
		SelectNodesInShape(*m_polygon, m_chunks_x * CHUNK_WIDTH, m_chunks_y * CHUNK_WIDTH, 0.0, 0.0, sdf::SDF_OPTION::NORMAL, m_invertedSdf);
	}
	if (ImGui::Button("select nodes in pw line")) {
		SelectNodesInShape(*m_pwLine, m_chunks_x * CHUNK_WIDTH, m_chunks_y * CHUNK_WIDTH, 0.0, m_pwLineRounding, sdf::SDF_OPTION::ROUNDED, m_invertedSdf);
	}
	if (ImGui::Button("Count selected nodes (DEBUG TOOL)")) {
		void *ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
		GridNode* data = static_cast<GridNode*>(ptr);
		int counted = 0;
		int convergedCount = 0;
		for (int i = 0; i < m_chunks_x * CHUNK_WIDTH; i++) {
			for (int j = 0; j < m_chunks_y * CHUNK_WIDTH; j++) {

				GridNode currNode = data[i * GRID_SIZE_Y + j];

				if (currNode.selected)
					counted++;
				if (currNode.converged)
					convergedCount++;
			}
		}

		std::cout << "counted = " << counted << std::endl;
		std::cout << "convergedCount = " << convergedCount << std::endl;
		glUnmapNamedBuffer(gridSSBO);
	}
	if (ImGui::Button("Clear selection of nodes")) {
		ClearNodesSelected(m_chunks_x * CHUNK_WIDTH, m_chunks_y * CHUNK_WIDTH);
	}
	static real accStr = 5.0;
	ImGui::InputReal("Acceleration Strength", &accStr, 0.5, 5.0);
	if (ImGui::Button("Compute nodal accelerations")) {
		CalculateNodalAccelerations(m_chunks_x * CHUNK_WIDTH, m_chunks_y * CHUNK_WIDTH, accStr);
	}
	if (ImGui::Button("Clear nodal accelerations")) {
		ClearNodalAcclerations(m_chunks_x * CHUNK_WIDTH, m_chunks_y * CHUNK_WIDTH);
	}

	ImGui::Checkbox("Node Selection Graphics", &m_nodeGraphicsActive);
	if (ImGui::Button("Select Node")) {
		m_selectNodeState = true;
	}
	ImGui::InputInt2("Grid Node:", m_node);
	if (ImGui::Button("Get node data") && m_paused) {
		UpdateNodeData();
	}
	if (ImGui::Button("Get node largest |rk|")) {
		real largest_norm_rk = 0.0;
		int node_i = 0, node_j = 0;
		void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);
		GridNode* data = static_cast<GridNode*>(ptr);
		for (int i = 0; i < GRID_SIZE_X * GRID_SIZE_Y; i++) {
			GridNode gn = data[i];
			/*if (!gn.converged) {
				converged = false;
			}*/
			if (gn.m > 0.0) {
				real cur_norm = glm::abs(glm::dot(gn.rk, gn.rk));
				if (cur_norm > largest_norm_rk) {
					largest_norm_rk = cur_norm;
					node_i = i / GRID_SIZE_Y;
					node_j = i % GRID_SIZE_Y;
				}
			}
		}
		glUnmapNamedBuffer(gridSSBO);
		std::cout << "largest norm is " << largest_norm_rk << " at node: (" << node_i << ", " << node_j << ")" << std::endl;
		m_node[0] = node_i;
		m_node[1] = node_j;
	}
	ImGui::Checkbox("View grid", &m_viewGrid);
	if (ImGui::CollapsingHeader("Grid Viewing Options")) {
		ImGui::Checkbox("View grid mass", &m_viewGridMass);
		if (ImGui::CollapsingHeader("View grid mass options")) {
			ImGui::InputReal("max node mass clamp", &m_maxNodeMassClamp, 1.0, 10.0, "%.1f");
			ImGui::InputReal("min node mass clamp", &m_minNodeMassClamp, 1.0, 10.0, "%.1f");
			ImGui::InputReal("min point size", &m_minNodeMassPointSize, 0.1, 1.0, "%.2f");
			ImGui::InputReal("max point size", &m_maxNodeMassPointSize, 0.1, 1.0, "%.2f");
			ImGui::InputInt("point size scaling option", &m_gridPointSizeScalingOption, 1);
			//ImGui::ListBox("Point size scaling option", &m_gpScalingOption, m_gpScalingOptions, 3);
		}
		ImGui::Checkbox("View grid vector", &m_viewGridVector);
		if (ImGui::CollapsingHeader("View grid vector options")) {
			ImGui::InputInt("vector option", &m_gridVectorOption);
			std::string vectorStr = "";
			switch (m_gridVectorOption) {
			case -1:
				vectorStr = "momentum";
				break;
			case 0:
				vectorStr = "velocity";
				break;
			case 1:
				vectorStr = "acceleration";
				break;
			case 2:
				vectorStr = "force";
				break;
			case 3:
				vectorStr = "residual velocity";
				break;
			case 4:
				vectorStr = "nodal acceleration";
				break;
			default:
				break;
			}
			vectorStr = "Vector vis: " + vectorStr;
			ImGui::Text(vectorStr.c_str());
			ImGui::InputReal("Max grid vector length", &m_maxGridVectorLength, 0.5, 5.0, "%.3f");
			ImGui::InputReal("Max grid vector visual length", &m_maxGridVectorVisualLength, 0.5, 1.0, "%.3f");
		}
	}
	ImGui::Checkbox("Marching squares", &m_viewMarchingSquares);
	ImGui::InputReal("Isomass", &m_isoMass, 0.5, 5.0);
	static float mscolor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	mscolor[0] = m_marchingSquaresColor.x;
	mscolor[1] = m_marchingSquaresColor.y;
	mscolor[2] = m_marchingSquaresColor.z;
	mscolor[3] = m_marchingSquaresColor.w;

	ImGui::ColorEdit4("MS Color", mscolor);
	m_marchingSquaresColor.x = mscolor[0];
	m_marchingSquaresColor.y = mscolor[1];
	m_marchingSquaresColor.z = mscolor[2];
	m_marchingSquaresColor.w = mscolor[3];

	if (ImGui::CollapsingHeader("Grid Node Data")) {
		glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
		glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);
		ImGui::DisplayNamedGlmRealColor("m", m_gn.m, max_color);
		ImGui::DisplayNamedGlmVecMixColor("v", m_gn.v, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("mv", m_gn.momentum, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("f", m_gn.force, min_color, max_color);
		ImGui::DisplayNamedGlmVecMixColor("nodal acc", m_gn.nodalAcceleration, min_color, max_color);
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
		ImGui::DisplayNamedBoolColor("selected", m_gn.selected, max_color, min_color);
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderMaterialPointViewer()
{
	ImGui::Begin("Material Point Viewer");

	ImGui::Checkbox("View point clouds", &m_viewPointClouds);

	if (ImGui::CollapsingHeader("Point Clouds")) {
		for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
			ImGui::Text(pointCloudPair.first.c_str());
		}
	}
	m_pointCloudSelect.resize(30);
	ImGui::InputText("Check point cloud", m_pointCloudSelect.data(), 30);
	std::string pointCloudSelectStr = std::string(m_pointCloudSelect.data());

	static int numPoints = 0;

	if (m_pointCloudMap.count(pointCloudSelectStr)) {
		numPoints = m_pointCloudMap[pointCloudSelectStr]->N;
	}
	else {
		numPoints = 0;
	}
	
	std::string numPointsStr = "N: " + std::to_string(numPoints);
	ImGui::Text(numPointsStr.c_str());

	static int pointIndex = 0;


	ImGui::InputInt("Point Index", &pointIndex);
	pointIndex = glm::min(glm::max(pointIndex, 0), numPoints - 1); // keep point index in bounds
	
	
	if (ImGui::Button("View Particles") && m_paused) {
		if (m_pointCloudMap.count(pointCloudSelectStr)) {
			void* ptr = glMapNamedBuffer(m_pointCloudMap[pointCloudSelectStr]->ssbo, GL_READ_ONLY);
			MaterialPoint* data = static_cast<MaterialPoint*>(ptr);
			m_mp = data[pointIndex];
			glUnmapNamedBuffer(m_pointCloudMap[pointCloudSelectStr]->ssbo);
		}
	}
	ImGui::InputReal("Max energy clamp (for coloring)", &m_maxEnergyClamp, 1.0, 1000.0, "%.1f");
	ImGui::InputReal("Min energy clamp (for coloring)", &m_minEnergyClamp, 1.0, 1000.0, "%.1f");
	ImGui::Checkbox("Visualize Energy", &m_visualizeEnergy);

	ImGui::InputReal("Max speed clamp (for coloring)", &m_maxSpeedClamp, 1.0, 1000.0, "%.1f");
	ImGui::InputReal("Min speed clamp (for coloring)", &m_minSpeedClamp, 1.0, 1000.0, "%.1f");
	ImGui::Checkbox("Visualize Speed", &m_visualizeSpeed);

	


	

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

void mpm::MpmEngine::RenderZoomWindow()
{
	ImGui::Begin("Zoom Window");
	ImGui::ColorEdit4("Background color", m_backgroundColor);
	ImGui::InputReal("Zoom Point x: ", &m_zoomPoint.x, 1.0, 10.0, "%.1f");
	ImGui::InputReal("Zoom Point y: ", &m_zoomPoint.y, 1.0, 10.0, "%.1f");
	ImGui::InputReal("Zoom Factor", &m_zoomFactor, 0.5, 2.0, "%.1f");
	ImGui::Checkbox("Show Zoom Border", &m_showZoomBorder);
	ImGui::Checkbox("Move Zoom Window", &m_movingZoomWindow);
	ImGui::Image(
		(void*)(intptr_t)m_zoomWindow->texture,
		ImVec2((float)m_zoomWindow->screen_dimensions.x, (float)m_zoomWindow->screen_dimensions.y),
		ImVec2(0, 1),
		ImVec2(1, 0),
		ImVec4(1,1,1,1),
		ImVec4(1,1,1,1)
	);


	ImGui::End();
}
