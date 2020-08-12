#include "MpmControlEngine.h"

void mpm::MpmControlEngine::GUI()
{
	if (m_imguiExternalForceController) ImGuiExternalForceController();
	if (m_imguiDeformationGradientController) ImGuiDeformationGradientController();
	if (m_imguiMaterialParameterController) ImGuiMaterialParameterController();
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

		ImGui::InputReal("Global Force x", &m_globalForce.x, 0.1, 1.0, "%.15f");
		ImGui::InputReal("Global Force y", &m_globalForce.y, 0.1, 1.0, "%.15f");

		if (ImGui::Button("Gravity")) {
			m_globalForce.y = -9.81;
		}

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

		if (ImGui::Button("Calculate Young's Modulus and Poisson's Ratio")) {
			materialParametersControl.CalculateYongeAndPoisson();
		}

		ImGui::InputReal("Critical Compression", &materialParametersControl.crit_c);
		ImGui::InputReal("Critical Stretch", &materialParametersControl.crit_s);
		ImGui::InputReal("Hardening coefficient", &materialParametersControl.hardening);

		ImGui::Checkbox("Set Selected", &setSelected);


		if (ImGui::Button("Slime lame parameters")) {
			materialParametersControl.lam = 2500.0;
			materialParametersControl.mew = 100.0;
		}


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