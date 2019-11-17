#include "MpmControlEngine.h"

void mpm::MpmControlEngine::GUI()
{
	if (m_imguiExternalForceController) ImGuiExternalForceController();
	if (m_imguiDeformationGradientController) ImGuiDeformationGradientController();
	if (m_imguiMaterialParameterController) ImGuiMaterialParameterController();
	if (m_imguiDeformationGradientSpaceTimeController) ImGuiDeformationGradientSpaceTimeController();
}

void mpm::MpmControlEngine::Menu()
{
	if (ImGui::BeginMenu("Control")) {
		if (ImGui::MenuItem("External Force Controller", "", m_imguiExternalForceController)) {
			m_imguiExternalForceController = !m_imguiExternalForceController;
		}
		ImGui::Separator();
		if (ImGui::MenuItem("Simple Deformation Gradient Controller", "", m_imguiDeformationGradientController)) {
			m_imguiDeformationGradientController = !m_imguiDeformationGradientController;
		}
		if (ImGui::MenuItem("Material Parameter Controller", "", m_imguiMaterialParameterController)) {
			m_imguiMaterialParameterController = !m_imguiMaterialParameterController;
		}
		if (ImGui::MenuItem("SpaceTime Deformation Gradient Controller", "", m_imguiDeformationGradientSpaceTimeController)) {
			m_imguiDeformationGradientSpaceTimeController = !m_imguiDeformationGradientSpaceTimeController;
		}
		ImGui::EndMenu();
	}
}



void mpm::MpmControlEngine::ImGuiExternalForceController()
{
	if (ImGui::Begin("MPM External Force Controller", &m_imguiExternalForceController)) {


		ImGui::InputReal("drag", &m_drag, 0.0001, 0.01, "%.4f");
		if (ImGui::Button("Clear drag")) {
			m_drag = 0.0;
		}

		ImGui::InputReal("Global Force x", &m_globalForce.x, 0.1, 1.0, "%.16f");
		ImGui::InputReal("Global Force y", &m_globalForce.y, 0.1, 1.0, "%.16f");

		if (ImGui::Button("Clear external forces")) {
			m_globalForce.x = 0.0;
			m_globalForce.y = 0.0;
		}

		ImGui::InputReal("Mouse power", &m_mousePower);

	}

	ImGui::End();
}

void mpm::MpmControlEngine::ImGuiDeformationGradientController() {
	if (ImGui::Begin("MPM Deformation Gradient Controller", &m_imguiDeformationGradientController)) {

		static std::string pointCloudSelectStr = "";
		m_mpmEngine->ImGuiSelectPointCloud(pointCloudSelectStr, "Select Point Cloud");


		static std::vector<mat2> controlFeVec = { mat2(1.0) };


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
				SetDeformationGradientsGLSL(pointCloudSelectStr, controlFeVec[i], mat2(1.0), setSelected);
			}
			std::string setAllFeStr = "Set all point cloud deformation gradients to " + currFeStr;
			if (ImGui::Button(setAllFeStr.c_str())) {
				for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {
					SetDeformationGradientsGLSL(pointCloudPair.first, controlFeVec[i], mat2(1.0), setSelected);
				}
			}
			std::string multFeStr = "Multiply '" + pointCloudSelectStr + "' deformation gradients by " + currFeStr;
			if (ImGui::Button(multFeStr.c_str())) {
				MultiplyDeformationGradientsGLSL(pointCloudSelectStr, controlFeVec[i], mat2(1.0), multSelected);
			}
			std::string multAllFeStr = "Multiply all point cloud deformation gradients by " + currFeStr;
			if (ImGui::Button(multAllFeStr.c_str())) {
				for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {
					MultiplyDeformationGradientsGLSL(pointCloudPair.first, controlFeVec[i], mat2(1.0), multSelected);
				}
			}
		}
	}
	ImGui::End();
}

void mpm::MpmControlEngine::ImGuiMaterialParameterController() {

	if (ImGui::Begin("MPM Material Parameter Controller", &m_imguiMaterialParameterController)) {

		static MaterialParameters materialParametersControl;
		static std::string currPointCloud = "";
		static std::string pointCloudSelectStr = "";
		static bool setSelected;
		m_mpmEngine->ImGuiSelectPointCloud(pointCloudSelectStr, "Select Point Cloud");

		if (currPointCloud != pointCloudSelectStr && m_mpmEngine->m_pointCloudMap.count(pointCloudSelectStr)) {
			currPointCloud = pointCloudSelectStr;
			materialParametersControl = m_mpmEngine->m_pointCloudMap[currPointCloud]->parameters;
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

		ImGui::Checkbox("Set Selected", &setSelected);

		std::string setCurrPointCloudStr = "Set '" + currPointCloud + "' lame parameters";
		if (ImGui::Button(setCurrPointCloudStr.c_str())) {
			SetLameParametersGLSL(currPointCloud, materialParametersControl.lam, materialParametersControl.mew, setSelected);
		}
		if (ImGui::Button("Set all point cloud lame parameters")) {
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {
				SetLameParametersGLSL(pointCloudPair.first, materialParametersControl.lam, materialParametersControl.mew, setSelected);
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

void mpm::MpmControlEngine::ImGuiDeformationGradientSpaceTimeController()
{
	if (ImGui::Begin("MPM Deformation Gradient Spacetime Controller (for CPU mode)", &m_imguiDeformationGradientSpaceTimeController)) {
		//static std::string targetPointCloudSelectStr = "";
		//static bool setSelected;
		//m_mpmEngine->ImGuiSelectPointCloud(targetPointCloudSelectStr, "Select for target point cloud");
		//if (ImGui::Button("Create target point cloud from selected")) {
		//	if (m_mpmEngine->m_pointCloudMap.count(targetPointCloudSelectStr)) {
		//		// copy constructor
		//		m_targetPointCloud = std::make_shared<PointCloud>(m_mpmEngine->m_pointCloudMap[targetPointCloudSelectStr]);
		//	}
		//}

		ImGui::NewLine();

		static std::string controlPointCloudSelectStr = "";
		m_mpmEngine->ImGuiSelectPointCloud(controlPointCloudSelectStr, "Select control point cloud");

		// USING NEW SPACETIME CONTROL FUNCTIONS
		if (ImGui::Button("Create mpm::control::ControlPointCloud from selected")) {
			if (m_mpmEngine->m_pointCloudMap.count(controlPointCloudSelectStr)) {
				m_controlPointCloud = std::make_shared<control::ControlPointCloud>(m_mpmEngine->m_pointCloudMap[controlPointCloudSelectStr]);
			}
		}
		if (ImGui::Button("Create target point cloud from control point cloud")) {
			if (m_controlPointCloud != nullptr) {
				m_targetPointCloud = std::make_shared<control::ControlPointCloud>(m_controlPointCloud);
				m_targetPointCloud->color = glm::highp_fvec4(1.f, 1.f, 0.f, 1.f);
			}
		}

		ImGui::NewLine(); ImGui::NewLine();

		ImGui::InputReal("dt", &m_mpmAlgorithmEngine->m_dt, 0.001, 1.0 / 60.0, "%.6f");
		if (ImGui::Button("Multiply dt by 2")) {
			m_mpmAlgorithmEngine->m_dt *= 2.0;
		}
		if (ImGui::Button("Divide dt by 2")) {
			m_mpmAlgorithmEngine->m_dt /= 2.0;
		}

		ImGui::NewLine(); ImGui::NewLine();

		if (ImGui::Button("Create control grid")) {
			m_controlGrid = std::make_shared<control::ControlGrid>(
				m_mpmEngine->m_chunks_x * m_mpmAlgorithmEngine->m_cppChunkX,
				m_mpmEngine->m_chunks_y * m_mpmAlgorithmEngine->m_cppChunkY);
		}

		if (m_controlGrid == nullptr) {
			ImGui::Text("No control grid set up");
		}
		else {
			std::string controlGridSizeStr = "Control grid size: " + std::to_string(m_controlGrid->grid_size_x) + "x" + std::to_string(m_controlGrid->grid_size_y);
			ImGui::Text(controlGridSizeStr.c_str());
		}

		ImGui::NewLine(); ImGui::NewLine();

		if (m_controlPointCloud != nullptr) {
			std::string controlPointCloudNStr = "Control point cloud N: " + std::to_string(m_controlPointCloud->controlPoints.size());
			ImGui::Text(controlPointCloudNStr.c_str());

			static int num_steps = 120;
			ImGui::InputInt("Num time steps", &num_steps);
			num_steps = glm::max(0, num_steps);

			if (m_controlGrid != nullptr && ImGui::Button("Run Forward Simulation")) {
				control::MPMForwardSimulation(m_controlPointCloud, m_controlGrid, m_globalForce, num_steps, m_mpmAlgorithmEngine->m_dt, true);
			}
		}
		else {
			ImGui::Text("No control point cloud");
		}


		if (m_targetPointCloud != nullptr) {
			std::string targetPointCloudNStr = "Target point cloud N: " + std::to_string(m_targetPointCloud->controlPoints.size());
			ImGui::Text(targetPointCloudNStr.c_str());

			ImGui::Checkbox("Render target point cloud", &m_renderTargetPointCloud);

			static real expansionFactor = 1.0;
			ImGui::InputReal("Expansion factor", &expansionFactor, 0.1, 0.5);
			if (ImGui::Button("Expand target point cloud")) {
				control::ExpandPointCloud(m_targetPointCloud, expansionFactor);
				m_targetPointCloud->MapToGPU();
			}
		}
		else {
			ImGui::Text("No target point cloud");
		}

		

		ImGui::NewLine(); ImGui::NewLine();

		//if (m_targetPointCloud != nullptr) {
		//	ImGui::Checkbox("Render target point cloud", &m_renderTargetPointCloud);


		//	static float targetPointCloudColor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
		//	targetPointCloudColor[0] = m_targetPointCloud->color.x;
		//	targetPointCloudColor[1] = m_targetPointCloud->color.y;
		//	targetPointCloudColor[2] = m_targetPointCloud->color.z;
		//	targetPointCloudColor[3] = m_targetPointCloud->color.w;

		//	ImGui::ColorEdit4("Target point cloud color", targetPointCloudColor);
		//	m_targetPointCloud->color.x = targetPointCloudColor[0];
		//	m_targetPointCloud->color.y = targetPointCloudColor[1];
		//	m_targetPointCloud->color.z = targetPointCloudColor[2];
		//	m_targetPointCloud->color.w = targetPointCloudColor[3];


		//	ImGui::NewLine();

		///*	static real expansionFactor = 1.0;
		//	ImGui::InputReal("Expansion factor", &expansionFactor, 0.1, 0.5);
		//	if (ImGui::Button("Expand target point cloud")) {
		//		ExpandPointCloud(m_targetPointCloud, expansionFactor);
		//	}*/


		//	/*static real loss = -1.0;
		//	if (controlPointCloud != nullptr && ImGui::Button("Compute Position Loss")) {
		//		loss = PositionLossFunction(controlPointCloud, m_targetPointCloud);
		//	}
		//	std::string lossStr = "Position loss function eval = " + std::to_string(loss);
		//	ImGui::Text(lossStr.c_str());*/

		//}

		ImGui::NewLine(); ImGui::NewLine();

		/*if (controlPointCloud != nullptr) {
			if (ImGui::Button("Save control point cloud points")) {
				SaveControlPointCloudOriginalPoints(controlPointCloud);
			}
			if (ImGui::Button("Reset control point cloud points")) {
				ResetControlPointCloudPointsToSaved(controlPointCloud);
			}

			ImGui::NewLine();
			
			

			static int num_steps = 120;
			ImGui::InputInt("Num time steps", &num_steps);
			num_steps = glm::max(0, num_steps);

			if (ImGui::Button("Run simulation")) {
				m_mpmAlgorithmEngine->RunMPMSimulationCPP(m_mpmAlgorithmEngine->m_dt, num_steps, true, true);
			}

			
		}

		ImGui::NewLine();

		if (controlPointCloud != nullptr && m_targetPointCloud != nullptr && controlPointCloud->N == m_targetPointCloud->N) {

			static int max_iters = 10;
			ImGui::InputInt("Max gradient descent iterations", &max_iters);

			static int num_steps = 120;
			ImGui::InputInt("Num time steps", &num_steps);

			static mat2 initialControlFe = mat2(1.0);
			ImGui::InputReal("Fe[0][0]: ", &initialControlFe[0][0], 0.1, 1.0, "%.6f");
			ImGui::InputReal("Fe[0][1]: ", &initialControlFe[0][1], 0.1, 1.0, "%.6f");
			ImGui::InputReal("Fe[1][0]: ", &initialControlFe[1][0], 0.1, 1.0, "%.6f");
			ImGui::InputReal("Fe[1][1]: ", &initialControlFe[1][1], 0.1, 1.0, "%.6f");
			ImGui::DisplayNamedGlmMatrixMixColor("Initial Control Set Fe: ", initialControlFe, glm::highp_fvec4(1.0f, 0.0f, 0.0f, 1.0f), glm::highp_fvec4(0.0f, 1.0f, 0.0f, 1.0f));

			

			if (ImGui::Button("Optimize deformation gradient")) {
				OptimizeSetDeformationGradient(controlPointCloud, m_targetPointCloud, initialControlFe, num_steps, max_iters);
			}

		}*/
		
	}
	ImGui::End();
}