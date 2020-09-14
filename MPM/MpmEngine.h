#pragma once

#include "Constants.h"
#include "EngineHeader.h"

#include "Shader.h"
#include "Shape.h"
#include "PointCloud.h"
#include "Grid.h"

#include "OpenGLScreen.h"

#include "MpmGeometryEngine.h"
#include "MpmControlEngine.h"
#include "MpmAlgorithmEngine.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <map>
#include <tuple>

//#define MPM_CR_DEBUG 1

namespace mpm {

	class MpmEngine {
	public:
		MpmEngine();
		~MpmEngine() { CleanupComputeShaderPipeline(); }

		bool InitComputeShaderPipeline();
		bool InitEngines();
		bool InitMpmSpace(int grid_dim_x, int grid_dim_y);

		bool CleanupComputeShaderPipeline();

		void Update();
		

		void ProcessKeyboardInput(GLFWwindow* window, real lag);
		void ProcessMouseInput(GLFWwindow* window, real lag);
		void HandleStates();

		// PUBLIC VARIABLES
		
		int m_chunks_y = 4;
		int m_chunks_x = 4;
		
		/******************** MOUSE STATES ********************/
		vec2 m_mouseGlobalScreen = vec2(0.0);
		vec2 m_mouseMpmRenderScreen = vec2(0.0);
		vec2 m_mouseMpmRenderScreenNormalized = vec2(0.0);
		vec2 m_mouseMpmRenderScreenGridSpace = vec2(0.0);
		vec4 m_mouseMpmRenderScreenGridSpaceFull = vec4(0.0);
		bool m_mouseMoved = false;
		bool m_leftButtonDown = false;
		bool m_midButtonDown = false;
		bool m_rightButtonDown = false;

		GLuint gridSSBO;
		GLuint VisualizeVAO;

		

		// MPM DATA STRUCTURES
		std::shared_ptr<Grid> m_grid;
		std::map<std::string, std::shared_ptr<PointCloud>> m_pointCloudMap;

		void InitGrid(int grid_dim_x, int grid_dim_y);
		
		// MPM data structure interaction
		std::string m_pointCloudViewSelectStr = "";
		void UpdatePointCloudData(std::string pointCloudStr);
		void UpdateNodeData();

		void MapCPUPointCloudsToGPU();
		void MapCPUPointCloudToGPU(std::shared_ptr<PointCloud> pointCloud);
		void MapCPUGridToGPU();
		void MapGPUPointCloudsToCPU();
		void MapGPUPointCloudToCPU(std::shared_ptr<PointCloud> pointCloud);
		void MapGPUGridToCPU();



		// counters for naming things in m_pointCloudMap
		unsigned int m_circleCount = 0;
		unsigned int m_rectCount = 0;
		unsigned int m_isoTriCount = 0;
		int m_pwLineCount = 0;
		int m_polygonCount = 0;

		float m_backgroundColor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
		float m_densityColor[4] = { 1.0f, 0.0f, 0.0f, 1.0f };
		float m_mediumDensityColor[4] = { 0.0f, 1.0f, 0.0f, 1.0f };
		float m_minDensityColor[4] = { 0.0f, 0.0f, 1.0f, 1.0f };

	private:

		// Other Engines
		std::shared_ptr<MpmGeometryEngine> m_mpmGeometryEngine = nullptr;
		std::shared_ptr<MpmControlEngine> m_mpmControlEngine = nullptr;
		std::shared_ptr<MpmAlgorithmEngine> m_mpmAlgorithmEngine = nullptr;
		

	


		//*** RENDERING FUNCTIONS ***//
	public:
		void Render();
		void RenderGUI();
		// re-used helper functions for imgui
		void ImGuiSelectPointCloud(std::string& pointCloudSelectStr, const std::string& selectMsg);
		void ImGuiDropDown(const std::string& combo_name, size_t& index, const std::vector<std::string>& string_vec);


		void RenderPointCloud(std::shared_ptr<PointCloud> pointCloud, vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pointShader);

	private:
		void RenderScreenShader(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen);
		void RenderPointClouds(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pointShader);
		void RenderGrid(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader);
		void RenderMarchingSquares(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader);
		void RenderGridBorder(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> borderShader);

	public:
		void RenderDensityField(vec2 zoomPoint, real zoomFactor, int binding, GLuint ssbo, bool controlMode, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> densityShader);
	private:
		//*** GUI FUNCTIONS ***//
		void ImGuiSimulationSettings();

		void ImGuiPointCloudSaver();
		void ImGuiPointCloudLoader();
		void ImGuiGridOptions();
		void ImGuiGridNodeViewer();
		void ImGuiMaterialPointViewer();
		void ImGuiEnergyViewer();

		
		

		// state variables for rendering different windows
		
		bool m_imguiZoomWindow = false;

		bool m_imguiMpmRenderWindow = true;

		bool m_imguiSimulationSettings = true;

		// grid
		bool m_imguiGridOptions = false;
		bool m_imguiGridNodeViewer = false;

		// material point
		bool m_imguiMaterialPointViewer = false;
		bool m_imguiEnergyViewer = false;
		

		// save/load
		bool m_imguiPointCloudSaver = false;
		bool m_imguiPointCloudLoader = false;

		/******************** ZOOM WINDOW ********************/
		void InitZoomWindow();
		void ImGuiZoomWindow();
		std::shared_ptr<ImGuiScreen> m_zoomWindow = nullptr;
		vec2 m_zoomWindowDims; // (width, height)
		real m_zoomFactor = 1.0;
		bool m_zoomState = false;
		bool m_showZoomBorder = true;
		bool m_movingZoomWindow = true;
		vec2 m_zoomPoint = vec2(0.0, 0.0); // ZOOM POINT IN GRID SPACE
		//vec2 m_zoomDim = vec2(GRID_SIZE_X, GRID_SIZE_Y);

		/******************** MPM RENDER WINDOW AND RENDERING ********************/
		void InitMpmRenderWindow();
		void ImGuiMpmRenderWindow();
		void MpmRender();
		void ZoomRender();

	public:
		std::shared_ptr<ImGuiScreen> m_mpmRenderWindow = nullptr;




		

		// RENDERING
		std::shared_ptr<StandardShader> m_pPointCloudShader = nullptr;

		bool m_viewGridDensity = false; // for gridDensityShader
		bool m_densitySharp = true;
	private:
		std::shared_ptr<StandardShader> m_pPointCloudVectorShader = nullptr;
		std::shared_ptr<StandardShader> m_mouseShader = nullptr;
		std::shared_ptr<StandardShader> m_zoomWindowShader = nullptr;
		std::shared_ptr<StandardShader> m_gridShader = nullptr;
		std::shared_ptr<StandardShader> m_gridShaderVector = nullptr;
		std::shared_ptr<StandardShader> m_gridShaderMarchingSquares = nullptr;
		std::shared_ptr<StandardShader> m_gridDensityShader = nullptr;
		


		
		real m_gridMaxMass = 4.0;
		real m_gridMediumMass = 2.0;
		real m_gridMinMass = 0.5;
		bool m_useColorSpectrum = true;

		std::shared_ptr<StandardShader> m_borderShader = nullptr;

		std::shared_ptr<OpenGLScreen> m_openGLScreen = nullptr;



		
		
		

		/******************** MATERIAL POINT VIEWER ********************/
		MaterialPoint m_mp = MaterialPoint(vec2(0.0), vec2(0.0), 0.0); // selecting material points
		double m_maxSpeedClamp = 25.0;
		double m_minSpeedClamp = 0.0;
		bool m_visualizeSpeed = true;
		double m_maxEnergyClamp = 100.0;
		double m_minEnergyClamp = 0.0;
		bool m_visualizeEnergy = true;
		bool m_viewPointClouds = true;
		
		
		/******************** GRID NODE VIEWER ********************/
		
		bool m_nodeGraphicsActive = false;
		int m_node[2] = { 26, 8 };
		bool m_viewGrid = true;
		bool m_viewGridMass = false;
		real m_maxNodeMassClamp = 40.0;
		real m_minNodeMassClamp = 1.0;
		real m_minNodeMassPointSize = 0.0;
		real m_maxNodeMassPointSize = 5.0;
		bool m_viewGridVector = false;
		enum class GRID_VECTOR_OPTION {
			MOMENTUM = 0,
			VELOCITY = 1,
			ACCELERATION = 2,
			FORCE = 3,
			RESIDUAL_VELOCITY = 4,
			NODAL_ACCELERATION = 5
		};
		std::vector<std::string> m_gridVectorOptionStrVec = {
			"momentum",
			"velocity",
			"acceleration",
			"force",
			"residual velocity",
			"nodal acceleration"
		};
		GRID_VECTOR_OPTION m_gridVectorOption = GRID_VECTOR_OPTION::FORCE; // VELOCITY = 0, ACCELERATION = 1, FORCE = 2, RESIDUAL = 3
		int m_gridPointSizeScalingOption = 0;
		real m_maxGridVectorLength = 25.0;
		real m_maxGridVectorVisualLength = 5.0;
		
		// marching squares
		bool m_viewMarchingSquares = false;
		double m_isoMass = 5.0;
		glm::highp_fvec4 m_marchingSquaresColor = glm::highp_fvec4(1.0f);


		// grid node control
		GridNode m_gn; // selecting grid node
		bool m_selectNodeState = false;

		/*void SelectNodesInShape(sdf::Shape& shape,
			const int gridDimX, const int gridDimY,
			const real inner_rounding, const real outer_rounding, 
			sdf::SDF_OPTION sdfOption,
			bool inverted);
		bool m_collectiveNodeSelectionGraphics = false;
		void ClearNodesSelected(const int gridDimX, const int gridDimY);
		void CalculateNodalAccelerations(const int gridDimX, const int gridDimY, real accStr);
		void ClearNodalAcclerations(const int gridDimX, const int gridDimY);*/


		
		/************* ENERGY VIEWER *************/
		double total_kinetic_energy = 0.0;
		double total_elastic_potential_energy = 0.0;
		double total_gravity_potential_energy = 0.0;

		double max_total_kinetic_energy = 0.0;
		double max_total_elastic_potential_energy = 0.0;
		double max_total_gravity_potential_energy = 0.0;

		double total_energy = 0.0;
		double max_total_energy = 0.0;




		


		
	
		// imgui stuff
		bool m_imguiGUI = true;
	};



}