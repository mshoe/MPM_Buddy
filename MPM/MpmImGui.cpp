#include "MpmEngine.h"
#include "imgui/imgui.h"
#include "imgui/imfilebrowser.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"

void mpm::MpmEngine::RenderGUI()
{
	static bool imguiImGuiDemo = false;
	static bool imguiImGuiStyleDemo = false;

	if (m_imguiGUI) {

		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu("File"))
			{
				if (ImGui::MenuItem("Save Point Cloud", "", m_imguiPointCloudSaver)) {
					m_imguiPointCloudSaver = !m_imguiPointCloudSaver;
				}
				if (ImGui::MenuItem("Load Point Cloud", "", m_imguiPointCloudLoader)) {
					m_imguiPointCloudLoader = !m_imguiPointCloudLoader;
				}
				if (ImGui::MenuItem("ImGui Demo", "", imguiImGuiDemo)) {
					imguiImGuiDemo = !imguiImGuiDemo;
				}
				if (ImGui::MenuItem("ImGui Style Demo", "", imguiImGuiStyleDemo)) {
					imguiImGuiStyleDemo = !imguiImGuiStyleDemo;
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
				if (ImGui::MenuItem("Redo", "CTRL+Y", true, false)) {}  // Disabled item
				ImGui::Separator();
				if (ImGui::MenuItem("Cut", "CTRL+X")) {}
				if (ImGui::MenuItem("Copy", "CTRL+C")) {}
				if (ImGui::MenuItem("Paste", "CTRL+V")) {}
				ImGui::EndMenu();
			}
			if (ImGui::BeginMenu("View")) {
				if (ImGui::MenuItem("Simulation Settings", "", m_imguiSimulationSettings)) {
					m_imguiSimulationSettings = !m_imguiSimulationSettings;
				}
				if (ImGui::MenuItem("MPM Window", "", m_imguiMpmRenderWindow)) {
					m_imguiMpmRenderWindow = !m_imguiMpmRenderWindow;
				}
				if (ImGui::MenuItem("Zoom Window", "", m_imguiZoomWindow)) {
					m_imguiZoomWindow = !m_imguiZoomWindow;
				}
				if (ImGui::MenuItem("Material Point Viewer", "", m_imguiMaterialPointViewer)) {
					m_imguiMaterialPointViewer = !m_imguiMaterialPointViewer;
				}
				if (ImGui::MenuItem("Grid Options", "", m_imguiGridOptions)) {
					m_imguiGridOptions = !m_imguiGridOptions;
				}
				if (ImGui::MenuItem("Grid Node Viewer", "", m_imguiGridNodeViewer)) {
					m_imguiGridNodeViewer = !m_imguiGridNodeViewer;
				}
				if (ImGui::MenuItem("Energy Viewer", "", m_imguiEnergyViewer)) {
					m_imguiEnergyViewer = !m_imguiEnergyViewer;
				}
				
				ImGui::EndMenu();
			}


			m_mpmAlgorithmEngine->Menu();
			m_mpmGeometryEngine->Menu();
			m_mpmControlEngine->Menu();

		
			ImGui::EndMainMenuBar();
		}

		if (m_imguiMpmRenderWindow) ImGuiMpmRenderWindow();
		
		
		
		m_mpmGeometryEngine->GUI();
		m_mpmControlEngine->GUI();
		m_mpmAlgorithmEngine->GUI();

		if (m_imguiSimulationSettings) ImGuiSimulationSettings();
		if (m_imguiGridOptions) ImGuiGridOptions();
		if (m_imguiGridNodeViewer) ImGuiGridNodeViewer();
		if (m_imguiMaterialPointViewer) ImGuiMaterialPointViewer();
		if (m_imguiZoomWindow) ImGuiZoomWindow();
		if (m_imguiPointCloudSaver) ImGuiPointCloudSaver();
		if (m_imguiPointCloudLoader) ImGuiPointCloudLoader();
		if (m_imguiEnergyViewer) ImGuiEnergyViewer();
		
		
		
		if (imguiImGuiDemo) { ImGui::ShowDemoWindow(); }
		if (imguiImGuiStyleDemo) { ImGui::ShowStyleEditor(); }
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



void mpm::MpmEngine::ImGuiSimulationSettings()
{
	if (ImGui::Begin("Simulation Settings", &m_imguiPointCloudSaver)) {

		static int grid_dims[2] = { 32, 64 };
		ImGui::InputInt2("Grid Dimensions", grid_dims);

		if (ImGui::Button("Create Grid")) {
			InitMpmSpace(grid_dims[0], grid_dims[1]);
		}

	}
	ImGui::End();
}

void mpm::MpmEngine::ImGuiPointCloudSaver()
{
	if (ImGui::Begin("Point Cloud Saver", &m_imguiPointCloudSaver)) {

		static std::string savePointCloudSelectStr = "";

		ImGuiSelectPointCloud(savePointCloudSelectStr, "Select Point Cloud");



		static char savePointCloudStr[50] = "";

		ImGui::InputText("Save point cloud file", savePointCloudStr, 50);

		if (ImGui::Button("Save selected point cloud to file")) {
			std::cout << "Saving to " << savePointCloudStr << "..." << std::endl;
			if (m_pointCloudMap.count(savePointCloudSelectStr)) {
				m_pointCloudMap[savePointCloudSelectStr]->SaveToFile(std::string("..\\SavedMPM\\") + savePointCloudStr + std::string(".mpm"));
			}
		}
	}
	ImGui::End();
}

void mpm::MpmEngine::ImGuiPointCloudLoader()
{
	if (ImGui::Begin("Point Cloud Loader", &m_imguiPointCloudLoader)) {
		/**** LOADING POINT CLOUDS ****/

		static ImGui::FileBrowser loadFileDialog;
		static std::string loadPointCloudStr = "";
		static char loadPointCloudName[50];

		if (ImGui::Button("Open point cloud file")) {
			loadFileDialog.Open();
			loadFileDialog.SetPwd("..\\SavedMPM");
		}

		loadFileDialog.Display();

		if (loadFileDialog.HasSelected())
		{
			loadPointCloudStr = loadFileDialog.GetSelected().string();
		}

		ImGui::Text(std::string(std::string("Selected point cloud file: ") + loadPointCloudStr).c_str());

		ImGui::InputText("Loaded point cloud name", loadPointCloudName, 50);
		if (ImGui::Button("Load point cloud from file")) {
			if (!loadFileDialog.HasSelected()) {
				std::cout << "no file selected" << std::endl;
			}
			else {
				m_pointCloudMap[std::string(loadPointCloudName)] = std::make_shared<PointCloud>();
				m_pointCloudMap[std::string(loadPointCloudName)]->LoadFromFile(loadPointCloudStr);
			}
		}

		/**** END - LOADING POINT CLOUDS ****/
	}
	ImGui::End();
}

void mpm::MpmEngine::ImGuiGridOptions()
{
	if (ImGui::Begin("Grid Options", &m_imguiGridOptions)) {

		
		if (ImGui::CollapsingHeader("Grid Viewing Options")) {
			ImGui::ColorEdit4("Background color", m_backgroundColor);
			ImGui::Checkbox("Node Selection Graphics", &m_nodeGraphicsActive);
			ImGui::Checkbox("View grid", &m_viewGrid);
			ImGui::Checkbox("View grid mass", &m_viewGridMass);
			ImGui::Checkbox("View grid vector", &m_viewGridVector);
			//ImGui::Checkbox("Marching squares", &m_viewMarchingSquares);
			ImGui::Checkbox("Density field", &m_viewGridDensity);
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

			/*if (ImGui::TreeNode("Marching Squares options")) {

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

			if (ImGui::TreeNode("Density Field Options")) {

				ImGui::Checkbox("Use color spectrum", &m_useColorSpectrum);
				ImGui::ColorEdit4("Max density color", m_densityColor);
				ImGui::ColorEdit4("Medium density color", m_mediumDensityColor);
				ImGui::ColorEdit4("Min density color", m_minDensityColor);
				ImGui::InputReal("Max grid mass", &m_gridMaxMass);
				ImGui::InputReal("Medium grid mass", &m_gridMediumMass);
				ImGui::InputReal("Min grid mass", &m_gridMinMass);
				ImGui::Checkbox("Sharp density field", &m_densitySharp);
				ImGui::TreePop();
			}*/
		}

		/*if (ImGui::CollapsingHeader("Experimental grid forces")) {

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
		}*/

		if (ImGui::CollapsingHeader("Misc. experimental")) {
			if (ImGui::Button("Get node largest |rk|")) {
				real largest_norm_rk = 0.0;
				int node_i = 0, node_j = 0;
				void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);
				GridNode* data = static_cast<GridNode*>(ptr);
				for (int i = 0; i < m_grid->grid_dim_x * m_grid->grid_dim_y; i++) {
					GridNode gn = data[i];
					/*if (!gn.converged) {
						converged = false;
					}*/
					if (gn.m > 0.0) {
						real cur_norm = glm::abs(glm::dot(gn.rk, gn.rk));
						if (cur_norm > largest_norm_rk) {
							largest_norm_rk = cur_norm;
							node_i = i / m_grid->grid_dim_y;
							node_j = i % m_grid->grid_dim_y;
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

void mpm::MpmEngine::ImGuiGridNodeViewer()
{
	if (ImGui::Begin("Grid Node Viewer", &m_imguiGridNodeViewer)) {
		if (ImGui::Button("Select Node")) {
			m_selectNodeState = true;
		}
		ImGui::InputInt2("Grid Node:", m_node);
		if (ImGui::Button("Get node data") && m_mpmAlgorithmEngine->m_paused) {
			UpdateNodeData();
		}
		m_gn.ImGuiDisplay();
	}
	ImGui::End();
}

void mpm::MpmEngine::ImGuiMaterialPointViewer()
{
	if (ImGui::Begin("Material Point Viewer", &m_imguiMaterialPointViewer)) {

		ImGui::Checkbox("View point clouds", &m_viewPointClouds);

		ImGuiSelectPointCloud(m_pointCloudViewSelectStr, "Select Point Cloud");

		if (ImGui::Button("Update point cloud data")) {
			UpdatePointCloudData(m_pointCloudViewSelectStr);
		}


		static std::string otherPointCloudStr = "";
		ImGuiSelectPointCloud(otherPointCloudStr, "Select other point cloud");

		if (ImGui::Button("Merge other point cloud to selected")) {
			if (m_pointCloudMap.count(m_pointCloudViewSelectStr) && m_pointCloudMap.count(otherPointCloudStr)) {
				m_pointCloudMap[m_pointCloudViewSelectStr]->points.reserve(m_pointCloudMap[m_pointCloudViewSelectStr]->points.size() +
																		   m_pointCloudMap[otherPointCloudStr]->points.size());
				m_pointCloudMap[m_pointCloudViewSelectStr]->N = m_pointCloudMap[m_pointCloudViewSelectStr]->points.size() +
					m_pointCloudMap[otherPointCloudStr]->points.size();
				m_pointCloudMap[m_pointCloudViewSelectStr]->points.insert(m_pointCloudMap[m_pointCloudViewSelectStr]->points.end(),
																		  m_pointCloudMap[otherPointCloudStr]->points.begin(),
																		  m_pointCloudMap[otherPointCloudStr]->points.end());
				

				glDeleteBuffers(1, &m_pointCloudMap[m_pointCloudViewSelectStr]->ssbo);
				m_pointCloudMap[m_pointCloudViewSelectStr]->GenPointCloudSSBO();
			}
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
		
		pointIndex = min(max(pointIndex, 0), max(0, numPoints - 1)); // keep point index in bounds


		/*if (ImGui::Button("View Particles") && m_paused) {

		}*/
		ImGui::InputReal("Max energy clamp (for coloring)", &m_maxEnergyClamp, 1.0, 1000.0, "%.1f");
		ImGui::InputReal("Min energy clamp (for coloring)", &m_minEnergyClamp, 1.0, 1000.0, "%.1f");
		ImGui::Checkbox("Visualize Energy", &m_visualizeEnergy);

		ImGui::InputReal("Max speed clamp (for coloring)", &m_maxSpeedClamp, 1.0, 1000.0, "%.1f");
		ImGui::InputReal("Min speed clamp (for coloring)", &m_minSpeedClamp, 1.0, 1000.0, "%.1f");
		ImGui::Checkbox("Visualize Speed", &m_visualizeSpeed);

		static bool calcDecomp = false;
		ImGui::Checkbox("Calculate decompositions", &calcDecomp);

		static bool calcdPdF = false;
		ImGui::Checkbox("Calc dPdF for fixed corotational elasticity", &calcdPdF);
		
		static bool calcVolumeRatio = false;
		ImGui::Checkbox("Calc volume ratio", &calcVolumeRatio);

		if (m_pointCloudMap.count(m_pointCloudViewSelectStr)) {
			m_mp = m_pointCloudMap[m_pointCloudViewSelectStr]->points[pointIndex];
		}




		if (ImGui::CollapsingHeader("Material Point")) {
			m_mp.ImGuiDisplay(calcDecomp, calcdPdF, calcVolumeRatio);
		}

		
	}
	ImGui::End();
}

void mpm::MpmEngine::ImGuiEnergyViewer()
{
	static bool kinetic_energy = true;
	static bool elastic_potential_energy = true;
	static bool gravity_potential_energy = true;

	static double total_kinetic_energy = 0.0;
	static double total_elastic_potential_energy = 0.0;
	static double total_gravity_potential_energy = 0.0;

	static double max_total_kinetic_energy = 0.0;
	static double max_total_elastic_potential_energy = 0.0;
	static double max_total_gravity_potential_energy = 0.0;
	
	static double total_energy = 0.0;
	static double max_total_energy = 0.0;

	if (ImGui::Begin("Energy Viewer", &m_imguiEnergyViewer)) {

		if (ImGui::Button("Clear max energy values")) {
			max_total_kinetic_energy = 0.0;
			max_total_elastic_potential_energy = 0.0;
			max_total_gravity_potential_energy = 0.0;
			max_total_energy = 0.0;
		}

		ImGui::Checkbox("kinetic energy", &kinetic_energy);
		ImGui::Checkbox("elastic potential energy", &elastic_potential_energy);
		ImGui::Checkbox("gravity potential energy", &gravity_potential_energy);
		

		static const int energy_values_size = 100;
		
		static float Kenergy_values[energy_values_size];
		static float EPenergy_values[energy_values_size];
		static float GPenergy_values[energy_values_size];
		static float total_energy_values[energy_values_size];
		
		static double refresh_time = 0.0;
		if (m_mpmAlgorithmEngine->m_paused || refresh_time == 0.0f)
			refresh_time = ImGui::GetTime();
		while (refresh_time < ImGui::GetTime()) // Create dummy data at fixed 60 hz rate for the demo
		{
			total_energy = 0.0;	
			
			if (kinetic_energy) {
				total_kinetic_energy = 0.0;
				for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
					total_kinetic_energy += pointCloudPair.second->ComputeMPKE();
				}
				total_energy += total_kinetic_energy;

				// O(n): push values to the front
				for (int i = 0; i <= energy_values_size - 2; i++) {
					Kenergy_values[i] = Kenergy_values[i + 1];
				}
				Kenergy_values[energy_values_size - 1] = (float)total_kinetic_energy;

				if (total_kinetic_energy > max_total_kinetic_energy) {
					max_total_kinetic_energy = total_kinetic_energy;
				}
			}

			if (elastic_potential_energy) {
				total_elastic_potential_energy = 0.0;
				for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
					/*total_elastic_potential_energy += pointCloudPair.second->ComputeElasticPotential();*/
					total_elastic_potential_energy += pointCloudPair.second->SumEPE();
				}

				total_energy += total_elastic_potential_energy;

				// O(n): push values to the front
				for (int i = 0; i <= energy_values_size - 2; i++) {
					EPenergy_values[i] = EPenergy_values[i + 1];
				}
				EPenergy_values[energy_values_size - 1] = (float)total_elastic_potential_energy;

				if (total_elastic_potential_energy > max_total_elastic_potential_energy) {
					max_total_elastic_potential_energy = total_elastic_potential_energy;
				}
			}

			if (gravity_potential_energy) {
				total_gravity_potential_energy = 0.0;
				for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
					total_gravity_potential_energy += pointCloudPair.second->ComputeGravitionalPotential();
				}

				total_energy += total_gravity_potential_energy;

				// O(n): push values to the front
				for (int i = 0; i <= energy_values_size - 2; i++) {
					GPenergy_values[i] = GPenergy_values[i + 1];
				}
				GPenergy_values[energy_values_size - 1] = (float)total_gravity_potential_energy;

				if (total_gravity_potential_energy > max_total_gravity_potential_energy) {
					max_total_gravity_potential_energy = total_gravity_potential_energy;
				}
			}

			// O(n): push values to the front
			for (int i = 0; i <= energy_values_size - 2; i++) {
				total_energy_values[i] = total_energy_values[i + 1];
			}
			total_energy_values[energy_values_size - 1] = (float)total_energy;

			if (total_energy > max_total_energy) {
				max_total_energy = total_energy;
			}

			refresh_time += 1.0/60;
			
		}


		

		

		
		if (kinetic_energy) {
			double totalGridKE = m_grid->ComputeKE();
			ImGui::DisplayNamedGlmRealColor("total grid kinetic energy", totalGridKE, glm::highp_fvec4(1.0));
			ImGui::DisplayNamedGlmRealColor("total kinetic energy", total_kinetic_energy, glm::highp_fvec4(1.0));

			/*double interpolation_error = totalGridKE - total_kinetic_energy;
			ImGui::DisplayNamedGlmRealColor("interpolation error", interpolation_error, glm::highp_fvec4(1.0));*/

			double deltaKEPoints = 0.0;
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
				deltaKEPoints += pointCloudPair.second->DeltaKEPoints();
			}
			ImGui::DisplayNamedGlmRealColor("Delta KE Points", deltaKEPoints, glm::highp_fvec4(1.0));

			double deltaSEPoints = 0.0;
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
				deltaSEPoints += pointCloudPair.second->DeltaSEPoints();
			}
			ImGui::DisplayNamedGlmRealColor("Delta SE Points", deltaKEPoints, glm::highp_fvec4(1.0));


			double algorithm_error = -deltaKEPoints - deltaSEPoints;
			ImGui::DisplayNamedGlmRealColor("Algorithm Error", algorithm_error, glm::highp_fvec4(1.0));

			ImGui::DisplayNamedGlmRealColor("max total kinetic energy", max_total_kinetic_energy, glm::highp_fvec4(0.0, 1.0, 0.0, 1.0));
			ImGui::PlotLines("Total Kinetic Energy", Kenergy_values, IM_ARRAYSIZE(Kenergy_values), 0, "", 0.0f, (float)max_total_kinetic_energy, ImVec2(0, 160));
		}

		if (elastic_potential_energy) {
			ImGui::DisplayNamedGlmRealColor("total elastic potential energy", total_elastic_potential_energy, glm::highp_fvec4(1.0));
			ImGui::DisplayNamedGlmRealColor("max total elastic potential energy", max_total_elastic_potential_energy, glm::highp_fvec4(0.0, 1.0, 0.0, 1.0));
			ImGui::PlotLines("Total Elastic Potential Energy", EPenergy_values, IM_ARRAYSIZE(EPenergy_values), 0, "", 0.0f, (float)max_total_elastic_potential_energy, ImVec2(0, 160));
		}

		if (gravity_potential_energy) {
			ImGui::DisplayNamedGlmRealColor("total gravitational potential energy", total_gravity_potential_energy, glm::highp_fvec4(1.0));
			ImGui::DisplayNamedGlmRealColor("max total gravitational potential energy", max_total_gravity_potential_energy, glm::highp_fvec4(0.0, 1.0, 0.0, 1.0));
			ImGui::PlotLines("Total Gravity Potential Energy", GPenergy_values, IM_ARRAYSIZE(GPenergy_values), 0, "", 0.0f, (float)max_total_gravity_potential_energy, ImVec2(0, 160));
		}

		ImGui::DisplayNamedGlmRealColor("total energy", total_energy, glm::highp_fvec4(1.0));
		ImGui::DisplayNamedGlmRealColor("max energy", max_total_energy, glm::highp_fvec4(0.0, 1.0, 0.0, 1.0));
		ImGui::PlotLines("Total Energy", total_energy_values, IM_ARRAYSIZE(total_energy_values), 0, "", 0.0f, (float)max_total_energy, ImVec2(0, 160));


		/*if (ImGui::Button("Compute Total Kinetic Energy")) {
			
		}*/
	}
	ImGui::End();
}



void mpm::MpmEngine::ImGuiSelectPointCloud(std::string& pointCloudSelectStr, const std::string& selectMsg)
{
	if (ImGui::BeginCombo(selectMsg.c_str(), pointCloudSelectStr.c_str())) {
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
