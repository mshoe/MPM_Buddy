#include "MpmControlEngine.h"

void mpm::MpmControlEngine::GUI()
{
	if (m_imguiExternalForceController) ImGuiExternalForceController();
	if (m_imguiDeformationGradientController) ImGuiDeformationGradientController();
	if (m_imguiMaterialParameterController) ImGuiMaterialParameterController();
	if (m_imguiDeformationGradientSpaceTimeController) ImGuiDeformationGradientSpaceTimeController();
	if (m_imguiControlPointViewer) ImGuiControlPointViewer();
	if (m_imguiControlGridViewer) ImGuiControlGridViewer();
	if (m_imguiGradientDescentPlot) ImGuiGradientDescentPlot();
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
		if (ImGui::MenuItem("Control Point Viewer", "", m_imguiControlPointViewer)) {
			m_imguiControlPointViewer = !m_imguiControlPointViewer;
		}
		if (ImGui::MenuItem("Control Grid Viewer", "", m_imguiControlGridViewer)) {
			m_imguiControlGridViewer = !m_imguiControlGridViewer;
		}
		if (ImGui::MenuItem("Gradient Descent Plot", "", m_imguiGradientDescentPlot)) {
			m_imguiGradientDescentPlot = !m_imguiGradientDescentPlot;
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
				m_stcg->InitControlPointCloud(m_mpmEngine->m_pointCloudMap[controlPointCloudSelectStr]);
			}
		}
		if (ImGui::Button("Create target point cloud from selected")) {
			if (m_mpmEngine->m_pointCloudMap.count(controlPointCloudSelectStr)) {
				m_stcg->InitTargetPointCloud(m_mpmEngine->m_pointCloudMap[controlPointCloudSelectStr]);
			}
		}
		if (ImGui::Button("Create target point cloud from control point cloud")) {
			if (m_stcg->controlPointCloud != nullptr) {
				m_stcg->InitTargetPointCloud(m_stcg->controlPointCloud);
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

		if (ImGui::Button("Small Circle")) {
			m_mpmGeometryEngine->SmallCircle();
		}

		if (ImGui::Button("32 x 32 grid")) {
			m_mpmEngine->m_chunks_x = 1;
			m_mpmEngine->m_chunks_y = 1;
			m_mpmAlgorithmEngine->m_cppChunkX = 32;
			m_mpmAlgorithmEngine->m_cppChunkY = 32;
		}



		static bool controlGridSizeChosen = false;

		

		if (ImGui::Button("Choose control grid size")) {
			m_stcg->SetGridSize(m_mpmEngine->m_chunks_x * m_mpmAlgorithmEngine->m_cppChunkX,
								m_mpmEngine->m_chunks_y * m_mpmAlgorithmEngine->m_cppChunkY);
			controlGridSizeChosen = true;
		}

		


		if (controlGridSizeChosen == false) {
			ImGui::Text("No control grid set up");
		}
		else {
			std::string controlGridSizeStr = "Control grid size: " + std::to_string(m_stcg->grid_size_x) + "x" + std::to_string(m_stcg->grid_size_y);
			ImGui::Text(controlGridSizeStr.c_str());
		}

		ImGui::NewLine(); ImGui::NewLine();

		static int num_steps = 240;
		ImGui::InputInt("Num time steps", &num_steps);
		num_steps = glm::max(0, num_steps);
		m_stcg->timeSteps = num_steps;

		static bool debugOutput = false;
		ImGui::Checkbox("debugOutput", &debugOutput);

		static int max_iters = 10;
		ImGui::InputInt("Max gradient descent iterations", &max_iters);

		static int max_lineSearchIters = 5;
		ImGui::InputInt("Max backtracking line search iterations", &max_lineSearchIters);

		static real initialFAlpha = 0.025;
		ImGui::InputReal("initialFAlpha", &initialFAlpha, 0.01, 0.05);

		static real initialMaterialAlpha = 1000.0;
		ImGui::InputReal("initialMaterialAlpha", &initialMaterialAlpha, 100.0, 1000.0);

		static bool setInitialFe = false;
		ImGui::Checkbox("Set initial Fe", &setInitialFe);

		static mat2 initialControlFe = mat2(1.0);
		ImGui::InputReal("Fe[0][0]: ", &initialControlFe[0][0], 0.1, 1.0, "%.6f");
		ImGui::InputReal("Fe[0][1]: ", &initialControlFe[0][1], 0.1, 1.0, "%.6f");
		ImGui::InputReal("Fe[1][0]: ", &initialControlFe[1][0], 0.1, 1.0, "%.6f");
		ImGui::InputReal("Fe[1][1]: ", &initialControlFe[1][1], 0.1, 1.0, "%.6f");
		ImGui::DisplayNamedGlmMatrixMixColor("Initial Control Set Fe: ", initialControlFe, glm::highp_fvec4(1.0f, 0.0f, 0.0f, 1.0f), glm::highp_fvec4(0.0f, 1.0f, 0.0f, 1.0f));

		static bool optimizeOnlyInitialF = true;
		ImGui::Checkbox("Optimize only initial F", &optimizeOnlyInitialF);

		static int optFrameOffset = 50;
		ImGui::InputInt("Optimization timestep offset (only optimize every .. frames)", &optFrameOffset);
		optFrameOffset = glm::max(optFrameOffset, 0);

		static size_t lossFunctionIndex = 0;
		m_mpmEngine->ImGuiDropDown("Optimization Loss Function", lossFunctionIndex, control::lossFunctionStrVec);
		control::LOSS_FUNCTION lossFunction = control::LOSS_FUNCTION(lossFunctionIndex);

		static int totalTemporalIterations = 1;
		ImGui::InputInt("Total Temporal Iterations", &totalTemporalIterations);

		static bool forceDescent = false;
		ImGui::Checkbox("Force gradient descent (skip line search)", &forceDescent);

		static bool reverseTimeOpt = false;
		ImGui::Checkbox("Reverse Time Optimization", &reverseTimeOpt);

		static real penalty = 1.0;
		ImGui::InputReal("Penalty weight", &penalty, 0.1, 1.0);

		static real tol = 1e-3;
		ImGui::InputReal("Tolerance", &tol, 0.001, 0.1);


		ImGui::NewLine(); ImGui::NewLine();

		if (ImGui::Button("Set up mass control environment")) {
			m_globalForce.y = 0.0;
			m_mpmAlgorithmEngine->m_algo_code = MpmAlgorithmEngine::MPM_ALGORITHM_CODE::CPP;
			m_mpmGeometryEngine->SmallCircle();
			m_mpmEngine->m_chunks_x = 1;
			m_mpmEngine->m_chunks_y = 1;
			m_mpmAlgorithmEngine->m_cppChunkX = 32;
			m_mpmAlgorithmEngine->m_cppChunkY = 32;
			m_stcg->SetGridSize(m_mpmEngine->m_chunks_x * m_mpmAlgorithmEngine->m_cppChunkX,
								m_mpmEngine->m_chunks_y * m_mpmAlgorithmEngine->m_cppChunkY);
			controlGridSizeChosen = true;
			optFrameOffset = 10;
			optimizeOnlyInitialF = false;
			max_iters = 25;
			max_lineSearchIters = 15;
			lossFunctionIndex = size_t(control::LOSS_FUNCTION::GRID_NODE_MASSES);
			lossFunction = control::LOSS_FUNCTION::GRID_NODE_MASSES;
		}

		ImGui::NewLine(); ImGui::NewLine();

		if (m_stcg->controlPointCloud != nullptr) {
			std::string controlPointCloudNStr = "Control point cloud N: " + std::to_string(m_stcg->controlPointCloud->controlPoints.size());
			ImGui::Text(controlPointCloudNStr.c_str());

			

			if (controlGridSizeChosen && ImGui::Button("Run Forward Simulation")) {
				m_stcg->InitSTCG();
				control::MPMForwardSimulation(m_stcg, m_globalForce, m_mpmAlgorithmEngine->m_dt, 0, true);
			}

			
			
		}
		else {
			ImGui::Text("No control point cloud");
			ImGui::NewLine();
		}


		if (m_stcg->targetPointCloud != nullptr && m_stcg->controlPointCloud != nullptr) {
			std::string targetPointCloudNStr = "Target point cloud N: " + std::to_string(m_stcg->targetPointCloud->controlPoints.size());
			ImGui::Text(targetPointCloudNStr.c_str());

			if (ImGui::Button("Set target point cloud mass to control")) {
				m_stcg->SetTargetPointCloudMassToControl();
			}
			

			static real expansionFactor = 1.0;
			ImGui::InputReal("Expansion factor", &expansionFactor, 0.1, 0.5);
			if (ImGui::Button("Expand target point cloud")) {
				control::ExpandPointCloud(m_stcg->targetPointCloud, expansionFactor);
				control::MapCPUControlPointCloudToGPU(m_stcg->targetPointCloud, m_stcg->targetSsbo);
			}

			static vec2 translation = vec2(0.0);
			ImGui::InputReal("translation.x", &translation.x, 0.1, 1.0);
			ImGui::InputReal("translation.y", &translation.y, 0.1, 1.0);
			ImGui::DisplayNamedGlmVecMixColor("translation", translation, glm::highp_fvec4(1.0f, 0.0f, 0.0f, 1.0f), glm::highp_fvec4(0.0f, 1.0f, 0.0f, 1.0f));
			if (ImGui::Button("Translate target point cloud")) {
				control::TranslatePointCloud(m_stcg->targetPointCloud, translation);
				control::MapCPUControlPointCloudToGPU(m_stcg->targetPointCloud, m_stcg->targetSsbo);
			}

			static mat2 transformMat = mat2(1.0);
			ImGui::InputReal("transform[0][0]: ", &transformMat[0][0], 0.1, 1.0, "%.6f");
			ImGui::InputReal("transform[0][1]: ", &transformMat[0][1], 0.1, 1.0, "%.6f");
			ImGui::InputReal("transform[1][0]: ", &transformMat[1][0], 0.1, 1.0, "%.6f");
			ImGui::InputReal("transform[1][1]: ", &transformMat[1][1], 0.1, 1.0, "%.6f");
			ImGui::DisplayNamedGlmMatrixMixColor("Transform: ", transformMat, glm::highp_fvec4(1.0f, 0.0f, 0.0f, 1.0f), glm::highp_fvec4(0.0f, 1.0f, 0.0f, 1.0f));
			if (ImGui::Button("Transform target point cloud")) {
				control::TransformPointCloud(m_stcg->targetPointCloud, transformMat);
				control::MapCPUControlPointCloudToGPU(m_stcg->targetPointCloud, m_stcg->targetSsbo);
			}
		}
		else {
			ImGui::Text("No target point cloud");
		}

		ImGui::NewLine(); ImGui::NewLine();

		if (m_stcg->controlPointCloud != nullptr && m_stcg->targetPointCloud != nullptr) {
			if (ImGui::Button("Optimize L(F)")) {
				control::OptimizeSetDeformationGradient(m_stcg, m_globalForce, m_mpmAlgorithmEngine->m_dt, 
														initialControlFe, optFrameOffset,
														num_steps, max_iters, max_lineSearchIters,
														lossFunction, forceDescent,
														penalty,
														initialFAlpha,
														optimizeOnlyInitialF, debugOutput);
			}

			if (ImGui::Button("Optimize L(F) in temporal order")) {
				control::OptimizeSetDeformationGradient_InTemporalOrder(m_stcg, m_globalForce, m_mpmAlgorithmEngine->m_dt,
														setInitialFe, initialControlFe, 
														optFrameOffset,
														num_steps, max_iters, max_lineSearchIters,
														totalTemporalIterations,
														lossFunction, forceDescent,
														reverseTimeOpt, penalty,
														initialFAlpha, initialMaterialAlpha,
														tol,
														optimizeOnlyInitialF, debugOutput);
			}
		}

		//static char outputPointCloudStr[50] = "";
		//ImGui::InputText("Output point cloud name", outputPointCloudStr, 50);

		if (ImGui::Button("Set control to final time step point cloud")) {
			if (!m_stcg->simStates.empty() && m_stcg->simStates.back()->pointCloud != nullptr) {
				std::cout << "set control point cloud to final time step point cloud" << std::endl;
				m_stcg->controlPointCloud = m_stcg->simStates.back()->pointCloud;
				//m_stcg->
			}
		}

		if (ImGui::Button("Set control defgrads to identity")) {
			if (m_stcg->controlPointCloud != nullptr) {
				std::cout << "set control point cloud defgrads to identity..." << std::endl;
				m_stcg->controlPointCloud->SetFsToIdentity();
				//m_stcg->
			}
		}
		
	
		ImGui::NewLine(); ImGui::NewLine();
		
	}
	ImGui::End();
}

void mpm::MpmControlEngine::ImGuiControlPointViewer()
{
	if (ImGui::Begin("Control Point Viewer", &m_imguiControlPointViewer)) {
		ImGui::Checkbox("Render control point cloud", &m_renderControlPointCloud);
		ImGui::Checkbox("Render target point cloud", &m_renderTargetPointCloud);

		ImGui::InputReal("Point circle radius", &m_pointCircleRadius, 0.01, 0.05);

		static float pointColor[4] = { 1.0, 0.0, 0.0, 1.0 };
		ImGui::ColorEdit4("point color", pointColor);
		if (ImGui::Button("Set control point cloud color") && m_stcg->controlPointCloud != nullptr) {
			m_stcg->controlPointCloud->color = glm::highp_fvec4(pointColor[0], pointColor[1], pointColor[2], pointColor[3]);
		}
		if (ImGui::Button("Set target point cloud color") && m_stcg->targetPointCloud != nullptr) {
			m_stcg->targetPointCloud->color = glm::highp_fvec4(pointColor[0], pointColor[1], pointColor[2], pointColor[3]);
		}

		ImGui::Checkbox("Render points", &m_renderPoints);
		ImGui::Checkbox("Render circles", &m_renderCircles);
		ImGui::Checkbox("Render deformation gradients", &m_renderDeformationGradients);
		ImGui::Checkbox("Render control deformation gradients", &m_renderControlDeformationGradients);

		static int numPoints = 0;

		if (m_stcg->controlPointCloud != nullptr) {
			numPoints = int(m_stcg->controlPointCloud->controlPoints.size());
		}
		else {
			numPoints = 0;
		}

		std::string numPointsStr = "N: " + std::to_string(numPoints);
		ImGui::Text(numPointsStr.c_str());

		static int numTimeSteps = 0;
		numTimeSteps = m_stcg->timeSteps;
		std::string numTimeStepsStr = "time steps: " + std::to_string(numTimeSteps);
		ImGui::Text(numTimeStepsStr.c_str());

		static control::ControlPoint mp;

		static int simStatesIndex = 0;
		if (ImGui::InputInt("Time step", &simStatesIndex)) {
			simStatesIndex = glm::min(glm::max(simStatesIndex, 0), numTimeSteps - 1);
			if (int(m_stcg->simStates.size()) - simStatesIndex > 0 &&
				m_stcg->simStates[simStatesIndex]->pointCloud != nullptr)
			{
				control::MapCPUControlPointCloudToGPU(m_stcg->simStates[simStatesIndex]->pointCloud, m_stcg->controlSsbo);
				control::MapCPUControlGridToGPU(m_stcg->simStates[simStatesIndex]->grid, m_stcg->gridSsbo);
			}
		}

		static int pointIndex = 0;
		if (ImGui::InputInt("Point Index", &pointIndex)) {
		}

		pointIndex = glm::min(glm::max(pointIndex, 0), numPoints - 1); // keep point index in bounds

		if (int(m_stcg->simStates.size()) - simStatesIndex > 0 &&
			m_stcg->simStates[simStatesIndex]->pointCloud != nullptr &&
			int(m_stcg->simStates[simStatesIndex]->pointCloud->controlPoints.size()) - pointIndex > 0)
		{


			mp = m_stcg->simStates[simStatesIndex]->pointCloud->controlPoints[pointIndex];
		}

		ImGui::InputInt("Animation frames per displayed frame", &m_simStateFramesPerFrame);
		m_simStateFramesPerFrame = glm::min(glm::max(m_simStateFramesPerFrame, 0), numTimeSteps - 1);

		ImGui::Text(std::string("Current time step: " + std::to_string(m_currSimState)).c_str());
		ImGui::Text(std::string("time: " + std::to_string(0.0 + m_currSimState * m_mpmAlgorithmEngine->m_dt)).c_str());
		ImGui::Checkbox("View animation", &m_animateSimStates);
		ImGui::Checkbox("Loop", &m_animateLoop);

		static real controlPCmass = 0.0;
		if (m_stcg->controlPointCloud != nullptr) 
		{
			controlPCmass = m_stcg->controlPointCloud->totalMass;
		}

		static real targetPCmass = 0.0;
		if (m_stcg->targetPointCloud != nullptr)
		{
			targetPCmass = m_stcg->targetPointCloud->totalMass;
		}

		static glm::highp_fvec4 maxColor = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);


		ImGui::DisplayNamedGlmRealColor("control PC mass", controlPCmass, maxColor);
		ImGui::DisplayNamedGlmRealColor("target PC mass", targetPCmass, maxColor);
		

		if (ImGui::CollapsingHeader("Control Point")) {
			mp.ImGuiDisplay();
		}


	}
	ImGui::End();
}

void mpm::MpmControlEngine::ImGuiControlGridViewer()
{

	if (ImGui::Begin("Control Grid Node Viewer", &m_imguiControlGridViewer)) {
		static glm::highp_fvec4 maxColor = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);

		static int numTimeSteps = 0;
		numTimeSteps = m_stcg->timeSteps;
		std::string numTimeStepsStr = "time steps: " + std::to_string(numTimeSteps);
		ImGui::Text(numTimeStepsStr.c_str());

		static int simStatesIndex = 0;
		
		if (ImGui::InputInt("Time step", &simStatesIndex)) {
			simStatesIndex = glm::min(glm::max(simStatesIndex, 0), numTimeSteps - 1);
		}

		bool validSimState = (int(m_stcg->simStates.size()) - simStatesIndex > 0);

		static bool viewTargetGrid = false;
		ImGui::Checkbox("View target grid", &viewTargetGrid);


		static std::shared_ptr<control::ControlGrid> grid = nullptr;

		if (viewTargetGrid) {
			grid = m_stcg->targetGrid;
		}
		else if (validSimState) {
			grid = m_stcg->simStates[simStatesIndex]->grid;
		}

		bool validGrid = (grid != nullptr) && validSimState;


		static real totalMass = 0.0;
		if (ImGui::Button("Compute total mass") && validGrid) {
			totalMass = grid->GetTotalMass();
		}
		ImGui::DisplayNamedGlmRealColor("Total mass", totalMass, maxColor);


		static int node[2] = { 0, 0 };
		ImGui::InputInt2("Grid Node:", node);
		node[0] = glm::min(glm::max(node[0], 0), m_stcg->grid_size_x - 1);
		node[1] = glm::min(glm::max(node[1], 0), m_stcg->grid_size_y - 1);

		static control::ControlGridNode gn;
		
		if (validGrid &&
			m_stcg->grid_size_x > 0 &&
			m_stcg->grid_size_y > 0)
		{
			gn = grid->Node(node[0], node[1]);
		}
		//gn = 

		


		gn.ImGuiDisplay();
	}
	ImGui::End();

}

void mpm::MpmControlEngine::ImGuiGradientDescentPlot()
{
	if (ImGui::Begin("Gradient Descent Loss Viewer", &m_imguiGradientDescentPlot)) {
		ImGui::PlotLines("Gradient Descent Loss", m_stcg->lossValues.data(), m_stcg->lossValues.size(), 0, (const char*)0, 3.402823466e+38F,3.402823466e+38F, ImVec2(0, 120));
	}
	ImGui::End();
}
