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
		MpmEngine() { 
			InitEngines();
			InitComputeShaderPipeline(); 
		}
		~MpmEngine() { CleanupComputeShaderPipeline(); }

		bool InitEngines();
		bool InitComputeShaderPipeline();
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

		bool m_paused = true;


		std::map<std::string, std::shared_ptr<PointCloud>> m_pointCloudMap;

		// counters for naming things in m_pointCloudMap
		unsigned int m_circleCount = 0;
		unsigned int m_rectCount = 0;
		unsigned int m_isoTriCount = 0;
		int m_pwLineCount = 0;
		int m_polygonCount = 0;
	private:

		// Other Engines
		std::shared_ptr<MpmGeometryEngine> m_mpmGeometryEngine = nullptr;
		std::shared_ptr<MpmControlEngine> m_mpmControlEngine = nullptr;
		

		enum class MPM_ALGORITHM_CODE {
			GLSL = 0,
			CPP = 1
		};

		MPM_ALGORITHM_CODE m_algo_code = MPM_ALGORITHM_CODE::GLSL;

	public: // geometry engine wants access to this
		void CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);
	
	private:
		/******************** MPM FUNCTIONS GLSL COMPUTE SHADER IMPLEMENTATION ********************/
		void MpmReset_GLSL();
		void MpmTimeStep_GLSL(real dt);
		void MpmTimeStepP2G_GLSL(real dt);
		void MpmTimeStepExplicitGridUpdate_GLSL(real dt);
		void MpmTimeStepG2P_GLSL(real dt);
		void MpmTimeStepSemiImplicitCRGridUpdate_GLSL(real dt);
		void MpmCRInit_GLSL(real dt);
		bool MpmCRStep_GLSL(real dt, real& L2_norm_rk, bool& L2_converged, bool& L_inf_converged);
		void MpmCREnd_GLSL(real dt);
		void CalculatePointCloudVolumes_GLSL(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);

		/******************** MPM FUNCTIONS CPU C++ IMPLEMENTATION ********************/
		void MpmReset_CPP();
		void MpmTimeStep_CPP(real dt);
		void MpmTimeStepP2G_CPP(real dt);
		void MpmTimeStepExplicitGridUpdate_CPP(real dt);
		void MpmTimeStepSemiImplicitGridUpdate_CPP(real dt, real beta);
		void MpmTimeStepG2P_CPP(real dt);

		

		bool m_semiImplicitCPP = false;
		double m_beta = 1.0;

		void GetPointCloudVolumesFromGPUtoCPU(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);
		
		void MapCPUPointCloudsToGPU();
		void MapCPUGridToGPU();


		//*** RENDERING FUNCTIONS ***//
	public:
		void Render();
	private:
		void RenderScreenShader(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen);
		void RenderPointClouds(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pointShader);
		void RenderGrid(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader);
		void RenderMarchingSquares(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader);
		void RenderGridBorder(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> borderShader);
		//*** GUI FUNCTIONS ***//
	public:
		void RenderGUI();
		// re-used helper functions for imgui
		void ImGuiSelectPointCloud(std::string& pointCloudSelectStr);
		void ImGuiDropDown(const std::string& combo_name, size_t& index, const std::vector<std::string>& string_vec);


	private:
		void ImGuiTimeIntegrator();
		void ImGuiMaterialParametersEditor();
		void ImGuiGridOptions();
		void ImGuiGridNodeViewer();
		void ImGuiMaterialPointViewer();
		void ImGuiCPUMode();

		

		// state variables for rendering different windows
		bool m_imguiTimeIntegrator = false;
		bool m_imguiZoomWindow = false;

		bool m_imguiMpmRenderWindow = true;

		// grid
		bool m_imguiGridOptions = false;
		bool m_imguiGridNodeViewer = false;

		// material point
		bool m_imguiMaterialPointViewer = false;
		bool m_imguiMaterialParametersEditor = false;

		// experimental
		bool m_imguiCPUMode = false;

		/******************** ZOOM WINDOW ********************/
		void InitZoomWindow();
		void ImGuiZoomWindow();
		std::shared_ptr<ImGuiScreen> m_zoomWindow = nullptr;
		vec2 m_zoomWindowDims; // (width, height)
		real m_zoomFactor = 1.0;
		bool m_zoomState = false;
		bool m_showZoomBorder = true;
		bool m_movingZoomWindow = true;
		vec2 m_zoomPoint = vec2(40.0, 50.0); // ZOOM POINT IN GRID SPACE
		vec2 m_zoomDim = vec2(GRID_SIZE_X, GRID_SIZE_Y);

		/******************** MPM RENDER WINDOW AND RENDERING ********************/
		void InitMpmRenderWindow();
		void ImGuiMpmRenderWindow();
		void MpmRender();
		void ZoomRender();
		std::shared_ptr<ImGuiScreen> m_mpmRenderWindow = nullptr;




		/********************SHADERS ********************/
		std::unique_ptr<ComputeShader> m_gReset = nullptr;
		std::unique_ptr<ComputeShader> m_p2gScatter = nullptr;
		std::unique_ptr<ComputeShader> m_p2gGather = nullptr;
		std::unique_ptr<ComputeShader> m_gUpdate = nullptr;
		std::unique_ptr<ComputeShader> m_g2pGather = nullptr;
		

		std::unique_ptr<ComputeShader> m_p2gCalcVolumes = nullptr;
		std::unique_ptr<ComputeShader> m_g2pCalcVolumes = nullptr;

		// RENDERING
		std::shared_ptr<StandardShader> m_pPointCloudShader = nullptr;
		std::shared_ptr<StandardShader> m_pPointCloudVectorShader = nullptr;
		std::shared_ptr<StandardShader> m_mouseShader = nullptr;
		std::shared_ptr<StandardShader> m_zoomWindowShader = nullptr;
		std::shared_ptr<StandardShader> m_gridShader = nullptr;
		std::shared_ptr<StandardShader> m_gridShaderVector = nullptr;
		std::shared_ptr<StandardShader> m_gridShaderMarchingSquares = nullptr;

		std::shared_ptr<StandardShader> m_borderShader = nullptr;

		std::shared_ptr<OpenGLScreen> m_openGLScreen = nullptr;



		// Implict time integration shaders
		std::unique_ptr<ComputeShader> m_p2g2pDeltaForce = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsInitPart1 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsInitPart2 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsInitPart3 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsStepPart1 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsStepPart2 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsConclusion = nullptr;
		
		

		/******************** MATERIAL POINT VIEWER ********************/
		std::string m_pointCloudViewSelectStr = "";
		MaterialPoint m_mp = MaterialPoint(vec2(0.0), vec2(0.0), 0.0); // selecting material points
		void UpdatePointCloudData(std::string pointCloudStr);
		double m_maxSpeedClamp = 25.0;
		double m_minSpeedClamp = 0.0;
		bool m_visualizeSpeed = true;
		double m_maxEnergyClamp = 100.0;
		double m_minEnergyClamp = 0.0;
		bool m_visualizeEnergy = true;
		bool m_viewPointClouds = true;
		
		
		/******************** GRID NODE VIEWER ********************/
		Grid m_grid;
		bool m_nodeGraphicsActive = false;
		int m_node[2] = { 26, 8 };
		bool m_viewGrid = false;
		bool m_viewGridMass = true;
		real m_maxNodeMassClamp = 40.0;
		real m_minNodeMassClamp = 1.0;
		real m_minNodeMassPointSize = 0.0;
		real m_maxNodeMassPointSize = 5.0;
		bool m_viewGridVector = true;
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
		void UpdateNodeData();
		/*void SelectNodesInShape(sdf::Shape& shape,
			const int gridDimX, const int gridDimY,
			const real inner_rounding, const real outer_rounding, 
			sdf::SDF_OPTION sdfOption,
			bool inverted);
		bool m_collectiveNodeSelectionGraphics = false;
		void ClearNodesSelected(const int gridDimX, const int gridDimY);
		void CalculateNodalAccelerations(const int gridDimX, const int gridDimY, real accStr);
		void ClearNodalAcclerations(const int gridDimX, const int gridDimY);*/


		/******************** CONJUGATE RESIDUALS FOR SEMI-IMPLICT TIME INTEGRATION ********************/
		bool m_semi_implicit_CR = false;
		real m_semi_implicit_ratio = 1.0;
		int m_max_conj_res_iter = 300;//30;// GRID_SIZE_X* GRID_SIZE_Y;
		bool m_check_L2_norm = false;
		bool m_check_L_inf_norm = false;
		real m_L2_norm_threshold = 0.001; // * number of active nodes ?
		real m_L_inf_norm_threshold = 1.0; // * 1.0 / node mass?;
		int m_cr_step = 0;
		bool m_pause_if_not_converged = true;

		/******************** TIME INTEGRATOR ********************/
		int m_timeStep = 0;
		real m_time = 0.0;
		real m_dt = 1.0 / 120.0;
		bool m_rt = true; // realtime

		// TRANSFER SCHEMES (PIC/RPIC/APIC)
		enum class TRANSFER_SCHEME {
			PIC = 0,
			RPIC = 1,
			APIC = 2
		};
		std::vector<std::string> m_transferSchemeStrVec = {
			"PIC",
			"RPIC (not working)",
			"APIC"
		};
		TRANSFER_SCHEME m_transferScheme = TRANSFER_SCHEME::APIC;

		/******************** MATERIAL PARAMETERS EDITOR ********************/
	public:
		MaterialParameters m_mpParameters;
		std::vector<MaterialParameters> m_energyModels = std::vector<MaterialParameters>(size_t(ENERGY_MODEL::Count), MaterialParameters());
		std::vector<std::string> m_energyModelsStrVec = {
			"Neohookean Elasticity",
			"Fixed Corotational Elasticity",
			"Snow (Stomakhin et. al 2013)"
		};
		ENERGY_MODEL m_comodel = ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY;
		void ChangeEnergyModel(ENERGY_MODEL);
		float m_backgroundColor[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
		
	
	private:
		// imgui stuff
		bool m_imguiGUI = true;
	};



}