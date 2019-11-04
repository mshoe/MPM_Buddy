#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"

void mpm::MpmEngine::RenderGUI()
{
	static bool renderImguiDemo = false;

	if (m_renderGUI) {

		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu("File"))
			{
				if (ImGui::MenuItem("ImGui Demo", "", renderImguiDemo)) {
					renderImguiDemo = !renderImguiDemo;
				}
				if (ImGui::MenuItem("Re-initialize shader pipeline")) {
					CleanupComputeShaderPipeline();
					InitComputeShaderPipeline();
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Edit"))
			{
				if (ImGui::MenuItem("Undo", "CTRL+Z")) {}
				if (ImGui::MenuItem("Redo", "CTRL+Y", true , false)) {}  // Disabled item
				ImGui::Separator();
				if (ImGui::MenuItem("Cut", "CTRL+X")) {}
				if (ImGui::MenuItem("Copy", "CTRL+C")) {}
				if (ImGui::MenuItem("Paste", "CTRL+V")) {}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Windows")) {
				if (ImGui::MenuItem("Time Integrator", "", m_renderTimeIntegrator)) {
					m_renderTimeIntegrator = !m_renderTimeIntegrator;
				}
				if (ImGui::MenuItem("Geometry Editor", "", m_renderGeometryEditor)) {
					m_renderGeometryEditor = !m_renderGeometryEditor;
				}
				if (ImGui::MenuItem("Material Parameters Editor", "", m_renderMaterialParametersEditor)) {
					m_renderMaterialParametersEditor = !m_renderMaterialParametersEditor;
				}
				if (ImGui::MenuItem("Material Point Viewer", "", m_renderMaterialPointViewer)) {
					m_renderMaterialPointViewer = !m_renderMaterialPointViewer;
				}
				if (ImGui::MenuItem("Zoom Window", "", m_renderZoomWindow)) {
					m_renderZoomWindow = !m_renderZoomWindow;
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Grid")) {
				if (ImGui::MenuItem("Grid Options", "", m_renderGridOptions)) {
					m_renderGridOptions = !m_renderGridOptions;
				}
				if (ImGui::MenuItem("Grid Node Viewer", "", m_renderGridNodeViewer)) {
					m_renderGridNodeViewer = !m_renderGridNodeViewer;
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Control")) {
				if (ImGui::MenuItem("External Force Controller", "", m_renderExternalForceController)) {
					m_renderExternalForceController = !m_renderExternalForceController;
				}
				ImGui::Separator();
				if (ImGui::MenuItem("Deformation Gradient Controller", "", m_renderDeformationGradientController)) {
					m_renderDeformationGradientController = !m_renderDeformationGradientController;
				}
				if (ImGui::MenuItem("Material Parameter Controller", "", m_renderMaterialParameterController)) {
					m_renderMaterialParameterController = !m_renderMaterialParameterController;
				}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("Experimental")) {
				if (ImGui::MenuItem("CPU Mode", "", m_renderCPUMode)) {
					m_renderCPUMode = !m_renderCPUMode;
				}
				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}

		//RenderWindowManager();
		if (m_renderTimeIntegrator) RenderTimeIntegrator();
		if (m_renderExternalForceController) RenderExternalForceController();
		if (m_renderDeformationGradientController) RenderDeformationGradientController();
		if (m_renderMaterialParameterController) RenderMaterialParameterController();
		if (m_renderGeometryEditor) RenderGeometryEditor();
		if (m_renderMaterialParametersEditor) RenderMaterialParametersEditor();
		if (m_renderGridOptions) RenderGridOptions();
		if (m_renderGridNodeViewer) RenderGridNodeViewer();
		if (m_renderMaterialPointViewer) RenderMaterialPointViewer();
		if (m_renderZoomWindow) RenderZoomWindow();
		if (m_renderCPUMode) RenderCPUMode();
		
		if (renderImguiDemo) { ImGui::ShowDemoWindow(); }
	}
}

//void mpm::MpmEngine::RenderWindowManager()
//{
//	ImGui::Begin("Window Manager");
//
//	ImGui::Checkbox("Time Integrator", &m_renderTimeIntegrator);
//	ImGui::Checkbox("External Force Controller", &m_renderExternalForceController);
//	ImGui::Checkbox("Geometry Editor", &m_renderGeometryEditor);
//	ImGui::Checkbox("Material Parameters Editor", &m_renderMaterialParametersEditor);
//	ImGui::Checkbox("Grid Node Viewer", &m_renderGridNodeViewer);
//	ImGui::Checkbox("Material Point Viewer", &m_renderMaterialPointViewer);
//	ImGui::Checkbox("Zoom Window", &m_renderZoomWindow);
//
//	static bool renderImguiDemo = false;
//	ImGui::Checkbox("ImGui Demo", &renderImguiDemo);
//
//	ImGui::End();
//}

void mpm::MpmEngine::RenderTimeIntegrator()
{
	if (ImGui::Begin("Time Integrator", &m_renderTimeIntegrator)) {

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

		static size_t transferScheme = size_t(TRANSFER_SCHEME::APIC);
		ImGuiDropDown("Transfer Scheme", transferScheme, m_transferSchemeStrVec);
		m_transferScheme = TRANSFER_SCHEME(transferScheme);



		ImGui::Checkbox("Paused", &m_paused);
		if (ImGui::Button("Advance") && m_paused) {
			MpmTimeStep_GLSL(m_dt);
			UpdatePointCloudData(m_pointCloudViewSelectStr);
			UpdateNodeData();
		}
		if (ImGui::Button("Advance 10") && m_paused) {
			for (int i = 0; i < 10; i++) {
				MpmTimeStep_GLSL(m_dt);
			}
			UpdatePointCloudData(m_pointCloudViewSelectStr);
			UpdateNodeData();
		}
		if (ImGui::Button("Advance 100") && m_paused) {
			for (int i = 0; i < 100; i++) {
				MpmTimeStep_GLSL(m_dt);
			}
			UpdatePointCloudData(m_pointCloudViewSelectStr);
			UpdateNodeData();
		}
		if (ImGui::Button("Reset")) {
			MpmReset_GLSL();	
		}
		ImGui::Text("MPM Algorithm Breakdown");
		if (ImGui::Button("P2G") && m_paused) {
			MpmTimeStepP2G_GLSL(m_dt);
			UpdatePointCloudData(m_pointCloudViewSelectStr);
			UpdateNodeData();
		}
		if (ImGui::Button("Explicit Grid Update") && m_paused) {
			MpmTimeStepExplicitGridUpdate_GLSL(m_dt);
			UpdatePointCloudData(m_pointCloudViewSelectStr);
			UpdateNodeData();
		}
		if (ImGui::Button("Semi-Implicit Grid Update") && m_paused) {
			MpmTimeStepSemiImplicitCRGridUpdate_GLSL(m_dt);
			UpdatePointCloudData(m_pointCloudViewSelectStr);
			UpdateNodeData();
		}
		if (ImGui::Button("G2P") && m_paused) {
			MpmTimeStepG2P_GLSL(m_dt);
			UpdatePointCloudData(m_pointCloudViewSelectStr);
			UpdateNodeData();
		}

		ImGui::Text("Conjugate residual steps (used after \"Explicit Grid Update\"");
		if (ImGui::Button("CR Init") && m_paused) {
			m_cr_step = 0;
			MpmCRInit_GLSL(m_dt);
			UpdatePointCloudData(m_pointCloudViewSelectStr);
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
			MpmCREnd_GLSL(m_dt);
			UpdatePointCloudData(m_pointCloudViewSelectStr);
			UpdateNodeData();
		}
		//ImGui::DisplayNamedBoolColor("CR Convergence", converged, )
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderExternalForceController()
{
	if (ImGui::Begin("MPM External Force Controller", &m_renderExternalForceController)) {


		ImGui::InputReal("drag", &m_drag, 0.0001, 0.01, "%.4f");

		ImGui::InputReal("Global Force x", &m_globalForce.x, 0.1, 1.0, "%.16f");
		ImGui::InputReal("Global Force y", &m_globalForce.y, 0.1, 1.0, "%.16f");

		ImGui::InputReal("Mouse power", &m_mousePower);

	}

	ImGui::End();
}

void mpm::MpmEngine::RenderDeformationGradientController() {
	if (ImGui::Begin("MPM Deformation Gradient Controller", &m_renderDeformationGradientController)) {

		static std::string pointCloudSelectStr = "";
		ImGuiSelectPointCloud(pointCloudSelectStr);

		
		static std::vector<mat2> controlFeVec = { mat2(1.0) };
		//mat2 tempFe = m_setFe;

		//ImGui::Text("Fe:");
		//ImGui::InputReal("Fe[0][0]: ", &tempFe[0][0], 0.1, 1.0, "%.6f");
		//ImGui::InputReal("Fe[0][1]: ", &tempFe[0][1], 0.1, 1.0, "%.6f");
		//ImGui::InputReal("Fe[1][0]: ", &tempFe[1][0], 0.1, 1.0, "%.6f");
		//ImGui::InputReal("Fe[1][1]: ", &tempFe[1][1], 0.1, 1.0, "%.6f");
		//// Make sure the Fe is non-singular
		//if (glm::determinant(tempFe) != 0.0) {
		//	m_setFe = tempFe;
		//}
		//ImGui::DisplayNamedGlmMatrixMixColor("Fe: ", m_setFe, glm::highp_fvec4(1.0f, 0.0f, 0.0f, 1.0f), glm::highp_fvec4(0.0f, 1.0f, 0.0f, 1.0f));

		/*if (ImGui::Button("Set Selection PC Deformation Gradients")) {
			SetDeformationGradients(pointCloudDefGradControlSelectStr, m_setFe, m_setFp);
		}
		if (ImGui::Button("Set All Deformation Gradients")) {
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
				SetDeformationGradients(pointCloudPair.first, m_setFe, m_setFp);
			}
		}*/

		std::string vecSizeStr = "Number of control deformation gradients: " + std::to_string(int(controlFeVec.size()));
		ImGui::Text(vecSizeStr.c_str());

		if (ImGui::Button("Clear control deformation gradient vector")) {
			controlFeVec.clear();
		}
		if (ImGui::Button("Add control deformation gradient")) {
			controlFeVec.push_back(mat2(1.0));
		}
		static bool multSelected = false;
		static bool setSelected = false;

		ImGui::Checkbox("Set Fe for selected points", &setSelected);
		ImGui::Checkbox("Mult Fe for selected points", &multSelected);

		static int Fselect = 0;

		ImGui::InputInt("multFe selection", &Fselect);

		Fselect = glm::max(glm::min(Fselect, int(controlFeVec.size()) - 1), 0);

		if (Fselect < controlFeVec.size() && Fselect >= 0) {
			ImGui::InputReal("Fe[0][0]: ", &controlFeVec[Fselect][0][0], 0.1, 1.0, "%.6f");
			ImGui::InputReal("Fe[0][1]: ", &controlFeVec[Fselect][0][1], 0.1, 1.0, "%.6f");
			ImGui::InputReal("Fe[1][0]: ", &controlFeVec[Fselect][1][0], 0.1, 1.0, "%.6f");
			ImGui::InputReal("Fe[1][1]: ", &controlFeVec[Fselect][1][1], 0.1, 1.0, "%.6f");
		}

		for (size_t i = 0; i < controlFeVec.size(); i++) {
			std::string currFeStr = "Fe(" + std::to_string(i) + ")";
			ImGui::DisplayNamedGlmMatrixMixColor("multFe: ", controlFeVec[i], glm::highp_fvec4(1.0f, 0.0f, 0.0f, 1.0f), glm::highp_fvec4(0.0f, 1.0f, 0.0f, 1.0f));

			std::string setFeStr = "Set '" + pointCloudSelectStr + "' deformation gradients to " + currFeStr;
			if (ImGui::Button(setFeStr.c_str())) {
				SetDeformationGradients(pointCloudSelectStr, controlFeVec[i], mat2(1.0), setSelected);
			}
			std::string setAllFeStr = "Set all point cloud deformation gradients to " + currFeStr;
			if (ImGui::Button(setAllFeStr.c_str())) {
				for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
					SetDeformationGradients(pointCloudPair.first, controlFeVec[i], mat2(1.0), setSelected);
				}
			}
			std::string multFeStr = "Multiply '" + pointCloudSelectStr + "' deformation gradients by " + currFeStr;
			if (ImGui::Button(multFeStr.c_str())) {
				MultiplyDeformationGradients(pointCloudSelectStr, controlFeVec[i], mat2(1.0), multSelected);
			}
			std::string multAllFeStr = "Multiply all point cloud deformation gradients by " + currFeStr;
			if (ImGui::Button(multAllFeStr.c_str())) {
				for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
					MultiplyDeformationGradients(pointCloudPair.first, controlFeVec[i], mat2(1.0), multSelected);
				}
			}	
		}
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderMaterialParameterController() {

	if (ImGui::Begin("MPM Material Parameter Controller")) {

		static MaterialParameters materialParametersControl;
		static std::string currPointCloud = "";
		static std::string pointCloudSelectStr = "";
		ImGuiSelectPointCloud(pointCloudSelectStr);

		if (currPointCloud != pointCloudSelectStr && m_pointCloudMap.count(pointCloudSelectStr)) {
			currPointCloud = pointCloudSelectStr;
			materialParametersControl = m_pointCloudMap[currPointCloud]->parameters;
		}

		ImGui::InputReal("Young's Modulus", &materialParametersControl.youngMod);
		ImGui::InputReal("Poisson's Ratio", &materialParametersControl.poisson);
		if (ImGui::Button("Calculate Lame Parameters")) {
			materialParametersControl.CalculateLameParameters();
		}
		ImGui::InputReal("Lame's 1st Parameter (lambda)", &materialParametersControl.lam);
		ImGui::InputReal("Lame 2nd Parameter (mew, shear modulus)", &materialParametersControl.mew);
		ImGui::InputReal("Critical Compression", &materialParametersControl.crit_c);
		ImGui::InputReal("Critical Stretch", &materialParametersControl.crit_s);
		ImGui::InputReal("Hardening coefficient", &materialParametersControl.hardening);


		std::string setCurrPointCloudStr = "Set '" + currPointCloud + "' material parameters";
		if (ImGui::Button(setCurrPointCloudStr.c_str())) {
			if (m_pointCloudMap.count(currPointCloud)) {
				m_pointCloudMap[currPointCloud]->parameters = materialParametersControl;
			}
		}
		if (ImGui::Button("Set all point cloud material parameters")) {
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
				pointCloudPair.second->parameters = materialParametersControl;
				//pointCloudPair.second->mew = m_mpParameters.youngMod / (2.f + 2.f * m_mpParameters.poisson);
				//pointCloudPair.second->lam = m_mpParameters.youngMod * m_mpParameters.poisson / ((1.f + m_mpParameters.poisson) * (1.f - 2.f * m_mpParameters.poisson));
			}
		}

		/*
		if (ImGui::Button("Set All Point Cloud mew and lam")) {
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
				pointCloudPair.second->mew = m_controlMaterialParameters.mew;
				pointCloudPair.second->lam = m_controlMaterialParameters.lam;
			}
		}*/
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderGeometryEditor()
{
	if (ImGui::Begin("Geometry Editor", &m_renderGeometryEditor)) {



		ImGui::Checkbox("Fixed point cloud", &m_fixedPointCloud);
		ImGui::Checkbox("Inverted SDF (DANGER)", &m_invertedSdf);

		ImGui::InputReal("Point Spacing", &m_particleSpacing, 0.01, 0.1, "%.2f");

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
		ImGui::DisplayNamedGlmRealColor("Number of vertices", real(m_polygon->vertices.size()), glm::highp_fvec4(1.0));
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
		ImGui::DisplayNamedGlmRealColor("Number of vertices", real(m_pwLine->vertices.size()), glm::highp_fvec4(1.0));
		ImGui::Text("PW Line vertices:");
		for (int i = 0; i < m_pwLine->vertices.size(); i++) {
			ImGui::DisplayGlmVec(m_pwLine->vertices[i]);
		}
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderMaterialParametersEditor()
{
	if (ImGui::Begin("Material Parameters Editor", &m_renderMaterialParametersEditor)) {
		//ImGui::Color
		ImGui::ColorEdit4("Color", m_color);
		ImGui::InputReal("Initial Velocity X", &m_initVelocity.x, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Initial Velocity Y", &m_initVelocity.y, 0.1, 1.0, "%.1f");

		//ImGui::InputInt3("Color", m_color);

		static size_t energy_model = size_t(m_comodel);
		ImGuiDropDown("Energy model", energy_model, m_energyModelsStrVec);
		if (m_comodel != ENERGY_MODEL(energy_model)) {
			ChangeEnergyModel(ENERGY_MODEL(energy_model));
		}
		switch (m_comodel) {
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

void mpm::MpmEngine::RenderGridOptions()
{
	if (ImGui::Begin("Grid Options", &m_renderGridOptions)) {

		std::string chunkWidthStr = "Chunk width: " + std::to_string(CHUNK_WIDTH);
		ImGui::Text(chunkWidthStr.c_str());

		ImGui::InputInt("# chunks (x)", &m_chunks_x, 1, 1);
		m_chunks_x = glm::clamp(m_chunks_x, 1, 4);
		ImGui::InputInt("# chunks (y)", &m_chunks_y, 1, 1);
		m_chunks_y = glm::clamp(m_chunks_y, 1, 4);

		
		if (ImGui::CollapsingHeader("Grid Viewing Options")) {
			ImGui::Checkbox("Node Selection Graphics", &m_nodeGraphicsActive);
			ImGui::Checkbox("View grid", &m_viewGrid);
			ImGui::Checkbox("View grid mass", &m_viewGridMass);
			ImGui::Checkbox("View grid vector", &m_viewGridVector);
			ImGui::Checkbox("Marching squares", &m_viewMarchingSquares);
			if (ImGui::TreeNode("View grid mass options")) {
				ImGui::InputReal("max node mass clamp", &m_maxNodeMassClamp, 1.0, 10.0, "%.1f");
				ImGui::InputReal("min node mass clamp", &m_minNodeMassClamp, 1.0, 10.0, "%.1f");
				ImGui::InputReal("min point size", &m_minNodeMassPointSize, 0.1, 1.0, "%.2f");
				ImGui::InputReal("max point size", &m_maxNodeMassPointSize, 0.1, 1.0, "%.2f");
				ImGui::InputInt("point size scaling option", &m_gridPointSizeScalingOption, 1);
				ImGui::TreePop();
				//ImGui::ListBox("Point size scaling option", &m_gpScalingOption, m_gpScalingOptions, 3);
			}
			
			if (ImGui::TreeNode("View grid vector options")) {
				static size_t gridVectorOption = size_t(m_gridVectorOption);
				ImGuiDropDown("Grid Vector", gridVectorOption, m_gridVectorOptionStrVec);
				m_gridVectorOption = GRID_VECTOR_OPTION(gridVectorOption);
				ImGui::InputReal("Max grid vector length", &m_maxGridVectorLength, 0.5, 5.0, "%.3f");
				ImGui::InputReal("Max grid vector visual length", &m_maxGridVectorVisualLength, 0.5, 1.0, "%.3f");
				ImGui::TreePop();
			}

			if (ImGui::TreeNode("Marching Squares options")) {

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
				ImGui::TreePop();
			}
		}

		if (ImGui::CollapsingHeader("Experimental grid forces")) {

			ImGui::Checkbox("Collective node selection graphics", &m_collectiveNodeSelectionGraphics);
			if (ImGui::Button("Select nodes in polygon")) {
				SelectNodesInShape(*m_polygon, m_chunks_x * CHUNK_WIDTH, m_chunks_y * CHUNK_WIDTH, 0.0, 0.0, sdf::SDF_OPTION::NORMAL, m_invertedSdf);
			}
			if (ImGui::Button("select nodes in pw line")) {
				SelectNodesInShape(*m_pwLine, m_chunks_x * CHUNK_WIDTH, m_chunks_y * CHUNK_WIDTH, 0.0, m_pwLineRounding, sdf::SDF_OPTION::ROUNDED, m_invertedSdf);
			}
			if (ImGui::Button("Count selected nodes (DEBUG TOOL)")) {
				void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
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
		}

		if (ImGui::CollapsingHeader("Misc. experimental")) {
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
		}
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderGridNodeViewer()
{
	if (ImGui::Begin("Grid Node Viewer", &m_renderGridNodeViewer)) {
		if (ImGui::Button("Select Node")) {
			m_selectNodeState = true;
		}
		ImGui::InputInt2("Grid Node:", m_node);
		if (ImGui::Button("Get node data") && m_paused) {
			UpdateNodeData();
		}
		m_gn.ImGuiDisplay();
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderMaterialPointViewer()
{
	if (ImGui::Begin("Material Point Viewer", &m_renderMaterialPointViewer)) {

		ImGui::Checkbox("View point clouds", &m_viewPointClouds);

		/*if (ImGui::CollapsingHeader("Point Clouds")) {
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
				ImGui::Text(pointCloudPair.first.c_str());
			}
		}
		m_pointCloudSelect.resize(30);
		ImGui::InputText("Check point cloud", m_pointCloudSelect.data(), 30);*/
		//static std::string pointCloudSelectStr;// = std::string(m_pointCloudSelect.data());

		ImGuiSelectPointCloud(m_pointCloudViewSelectStr);

		if (ImGui::Button("Update point cloud data")) {
			UpdatePointCloudData(m_pointCloudViewSelectStr);
		}

		static int numPoints = 0;

		if (m_pointCloudMap.count(m_pointCloudViewSelectStr)) {
			numPoints = int(m_pointCloudMap[m_pointCloudViewSelectStr]->N);
		}
		else {
			numPoints = 0;
		}

		std::string numPointsStr = "N: " + std::to_string(numPoints);
		ImGui::Text(numPointsStr.c_str());

		static int pointIndex = 0;


		ImGui::InputInt("Point Index", &pointIndex);
		pointIndex = glm::min(glm::max(pointIndex, 0), numPoints - 1); // keep point index in bounds


		/*if (ImGui::Button("View Particles") && m_paused) {

		}*/
		ImGui::InputReal("Max energy clamp (for coloring)", &m_maxEnergyClamp, 1.0, 1000.0, "%.1f");
		ImGui::InputReal("Min energy clamp (for coloring)", &m_minEnergyClamp, 1.0, 1000.0, "%.1f");
		ImGui::Checkbox("Visualize Energy", &m_visualizeEnergy);

		ImGui::InputReal("Max speed clamp (for coloring)", &m_maxSpeedClamp, 1.0, 1000.0, "%.1f");
		ImGui::InputReal("Min speed clamp (for coloring)", &m_minSpeedClamp, 1.0, 1000.0, "%.1f");
		ImGui::Checkbox("Visualize Speed", &m_visualizeSpeed);

		ImGui::Checkbox("Visuzelize seslected points", &m_visualizeSelected);
		static float selectColor[4] = { 1.0f, 1.0f, 0.0f, 1.0f };
		selectColor[0] = m_pointSelectColor.x;
		selectColor[1] = m_pointSelectColor.y;
		selectColor[2] = m_pointSelectColor.z;
		selectColor[3] = m_pointSelectColor.w;
		ImGui::ColorEdit4("Selected points color", selectColor);

		if (ImGui::Button("Select points in polygon")) {
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
				SelectPointsInPolygon(pointCloudPair.first);
			}
		}

		if (m_pointCloudMap.count(m_pointCloudViewSelectStr)) {
			m_mp = m_pointCloudMap[m_pointCloudViewSelectStr]->points[pointIndex];
		}




		if (ImGui::CollapsingHeader("Material Point")) {
			m_mp.ImGuiDisplay();
		}

		
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderZoomWindow()
{
	if (ImGui::Begin("Zoom Window", &m_renderZoomWindow)) {
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
			ImVec4(1, 1, 1, 1),
			ImVec4(1, 1, 1, 1)
		);
	}
	ImGui::End();
}

void mpm::MpmEngine::RenderCPUMode()
{
	if (ImGui::Begin("CPU Mode", &m_renderCPUMode)) {
		static bool cpu_mode = false;
		ImGui::Checkbox("CPU Mode", &cpu_mode);
		if (cpu_mode) {
			m_algo_code = MPM_ALGORITHM_CODE::CPP;
		}
		else {
			m_algo_code = MPM_ALGORITHM_CODE::GLSL;
		}

		if (ImGui::Button("32 x 32 grid")) {
			m_chunks_x = 1;
			m_chunks_y = 1;
		}
		if (ImGui::Button("Small circle")) {
			m_circle_r = 2;
			m_particleSpacing = 0.5;
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

		ImGui::Checkbox("Paused", &m_paused);
		
		ImGui::Checkbox("Semi Implict Time Integration", &m_semiImplicitCPP);
		ImGui::InputDouble("Implicit Ratio (beta)", &m_beta, 0.05, 0.1);
		m_beta = glm::max(glm::min(1.0, m_beta), 0.0);

		if (ImGui::Button("Advance") && m_paused) {
			MpmTimeStep_CPP(m_dt);
			MapCPUPointCloudsToGPU();
			MapCPUGridToGPU();
		}
		
		if (ImGui::Button("Reset")) {
			MpmReset_CPP();
		}


	}
	ImGui::End();
}

void mpm::MpmEngine::ImGuiSelectPointCloud(std::string& pointCloudSelectStr)
{
	if (ImGui::BeginCombo("Select Point Cloud", pointCloudSelectStr.c_str())) {
		for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
			bool is_selected = pointCloudPair.first == pointCloudSelectStr;
			if (ImGui::Selectable(pointCloudPair.first.c_str(), is_selected)) {
				if (!is_selected) {
					pointCloudSelectStr = pointCloudPair.first;
					UpdatePointCloudData(pointCloudSelectStr);
				}
			}
			if (is_selected) {
				ImGui::SetItemDefaultFocus();
			}
		}
		ImGui::EndCombo();
	}
}

void mpm::MpmEngine::ImGuiDropDown(const std::string &combo_name, size_t& index, const std::vector<std::string>& string_vec)
{
	if (ImGui::BeginCombo(combo_name.c_str(), string_vec[index].c_str())) {

		for (size_t i = 0; i < string_vec.size(); i++) {

			bool is_selected = (i == index);
			if (ImGui::Selectable(string_vec[i].c_str(), is_selected)) {
				index = i;
			}
			if (is_selected) {
				ImGui::SetItemDefaultFocus();
			}
		}
		ImGui::EndCombo();
	}
}
