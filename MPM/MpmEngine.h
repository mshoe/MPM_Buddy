#pragma once

#include "Constants.h"

#include "Engine.h"

#include "Shader.h"
#include "Shape.h"
#include "PointCloud.h"
#include "Grid.h"

#include "OpenGLScreen.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <tuple>

//#define MPM_CR_DEBUG 1

namespace mpm {

	class MpmEngine : public Engine {
	public:
		MpmEngine() {}
		~MpmEngine() { CleanupComputeShaderPipeline(); }

		bool InitComputeShaderPipeline();
		bool CleanupComputeShaderPipeline();

		void Update();
		void Render();
		void RenderScreenShader(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen);
		void RenderPointClouds(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pointShader);
		void RenderGrid(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader);
		void RenderGUI();

		void ProcessKeyboardInput(GLFWwindow* window, real lag);
		void ProcessMouseInput(GLFWwindow* window, real lag);
		void HandleStates();


	private:

		//*** MPM FUNCTIONS ***//
		void MpmTimeStep(real dt);
		void MpmTimeStepP2G(real dt);
		void MpmTimeStepExplicitGridUpdate(real dt);
		void MpmTimeStepSemiImplicitGridUpdate(real dt);
		void MpmTimeStepG2P(real dt);
		void MpmCRInit(real dt);
		bool MpmCRStep(real dt, real& L2_norm_rk, bool& L2_converged, bool& L_inf_converged);
		void MpmCREnd(real dt);
		void CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);
		void SetReferenceConfig(std::string pointCloudID);

		//*** GUI FUNCTIONS ***//
		void RenderTimeIntegrator();
		void RenderForceController();
		void RenderGeometryEditor();
		void RenderGridNodeViewer();
		void RenderMaterialPointViewer();
		void RenderZoomWindow();

		//*** ZOOM WINDOW ***//
		void InitZoomWindow();
		std::shared_ptr<ImGuiScreen> m_zoomWindow = nullptr;
		GLuint m_zoom_VAO, m_zoom_VBO, m_zoom_EBO;
		vec2 m_zoomWindowDims; // (width, height)
		real m_zoomFactor = 1.0;
		bool m_zoomState = false;
		bool m_showZoomBorder = true;
		bool m_movingZoomWindow = true;
		vec2 m_zoomPoint = vec2(40.0, 50.0); // ZOOM POINT IN GRID SPACE
		vec2 m_zoomDim = vec2(GRID_SIZE_X, GRID_SIZE_Y);

		//*** GEOMETRY EDITOR FUNCTIONS ***//
		std::shared_ptr<PointCloud> GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
			const real gridDimX, const real gridDimY,
			const real inner_rounding, const real outer_rounding, 
			const MaterialParameters &parameters,
			const GLuint comodel, bool useHollow,
			vec2 initialVelocity, glm::highp_fvec4 color);
		void ClearCreateStates() {
			m_createCircleState = false;
			m_createRectState = false;
			m_createIsoTriState = false;
		}

		void GenPointCloudLineDivider();
		real m_line_m = 2.0;
		real m_line_b = -50.0;
		real m_line2_m = -2.0;
		real m_line2_b = 50.0;
		


		void PrintGridData();


		//*** SHADERS ***//
		std::unique_ptr<ComputeShader> m_gReset = nullptr;
		std::unique_ptr<ComputeShader> m_p2gScatter = nullptr;
		std::unique_ptr<ComputeShader> m_p2gGather = nullptr;
		std::unique_ptr<ComputeShader> m_gUpdate = nullptr;
		std::unique_ptr<ComputeShader> m_g2pGather = nullptr;
		

		std::unique_ptr<ComputeShader> m_p2gCalcVolumes = nullptr;
		std::unique_ptr<ComputeShader> m_g2pCalcVolumes = nullptr;

		std::unique_ptr<ComputeShader> m_pSetReferenceConfig = nullptr;

		// RENDERING
		std::shared_ptr<StandardShader> m_pPointCloudShader = nullptr;
		std::shared_ptr<StandardShader> m_pPointCloudVectorShader = nullptr;
		std::shared_ptr<StandardShader> m_mouseShader = nullptr;
		std::shared_ptr<StandardShader> m_zoomWindowShader = nullptr;
		std::shared_ptr<StandardShader> m_gridShader = nullptr;
		std::shared_ptr<StandardShader> m_gridShaderVector = nullptr;



		std::shared_ptr<OpenGLScreen> m_openGLScreen = nullptr;
		vec4 m_mpm_mouse = vec4(0.0);
		vec4 m_mouse = vec4(0.0);
		bool m_leftButtonDown = false;
		bool m_midButtonDown = false;
		bool m_rightButtonDown = false;

		

		// Implict time integration shaders
		std::unique_ptr<ComputeShader> m_p2g2pDeltaForce = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsInitPart1 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsInitPart2 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsInitPart3 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsStepPart1 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsStepPart2 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsConclusion = nullptr;
		
		GLuint gridSSBO;
		GLuint VisualizeVAO;

		// MATERIAL POINT VISUALIZATION STUFF
		MaterialPoint m_mp; // selecting material points
		double m_maxEnergyClamp = 100.0;
		double m_minEnergyClamp = 0.0;
		bool m_visualizeEnergy = true;
		bool m_viewPointClouds = true;
		
		// GRID VISUALIZATION STUFF
		Grid m_grid;
		bool m_nodeGraphicsActive = false;
		int m_node[2] = { 26, 8 };
		bool m_viewGrid = true;
		bool m_viewGridMass = true;
		real m_maxNodeMassClamp = 40.0;
		real m_minNodeMassClamp = 1.0;
		real m_minNodeMassPointSize = 0.0;
		real m_maxNodeMassPointSize = 5.0;
		bool m_viewGridVector = true;
		int m_gridVectorOption = 2; // VELOCITY = 0, ACCELERATION = 1, FORCE = 2, RESIDUAL = 3
		int m_gridPointSizeScalingOption = 0;
		real m_maxGridVectorLength = 25.0;
		real m_maxGridVectorVisualLength = 5.0;
		GridNode m_gn; // selecting grid node
		bool m_selectNodeState = false;
		void UpdateNodeData();


		real m_drag = 0.5;
		vec2 m_globalForce = vec2(0.0);
		//real m_set_dt = 1.0 / 120.0;
		real m_dt = 1.0 / 120.0;
		bool m_paused = true;
		bool m_implicit = false;
		real m_implicit_ratio = 1.0;
		int m_max_conj_res_iter = 300;//30;// GRID_SIZE_X* GRID_SIZE_Y;
		bool m_check_L2_norm = false;
		bool m_check_L_inf_norm = false;
		real m_L2_norm_threshold = 0.001; // * number of active nodes ?
		real m_L_inf_norm_threshold = 1.0; // * 1.0 / node mass?;
		int m_cr_step = 0;
		bool m_pause_if_not_converged = true;

		//int m_pointCloudSelect = 0;
		int m_timeStep = 0;
		real m_time = 0.0;
		bool m_rt = true;

		GLreal m_mousePower = 25.0;


		// Reorganize this stuff
		unsigned int m_circleCount = 0;
		unsigned int m_rectCount = 0;
		unsigned int m_isoTriCount = 0;
		unsigned int m_lineDivCount = 0;

		//std::vector<PointCloud> m_pointClouds;
		std::unordered_map<std::string, std::shared_ptr<PointCloud>> m_pointCloudMap;
		std::vector<char> m_pointCloudSelect;




		
		MaterialParameters m_mpParameters;
		MaterialParameters m_neoHookeanParameters;
		MaterialParameters m_fixedCorotatedParameters;
		MaterialParameters m_simpleSnowParameters;

		real m_lam = 0.0;
		real m_mew = 0.0;


		GLuint m_comodel = FIXED_COROTATIONAL_ELASTICITY;
		void ChangeMaterialParameters(GLuint);
		

		float m_color[4] = { 1.0f, 0.0f, 0.0f, 1.0f}; // color needs to be float
		vec2 m_initVelocity = vec2(0.0);
		
		bool m_createCircleState = false;
		real m_circle_r = 5.0;
		real m_circle_inner_radius = 0.0;
		real m_circle_rounding = 0.0;

		bool m_createRectState = false;
		real m_rect_b = 3.0;
		real m_rect_h = 3.0;
		real m_rect_inner_radius = 0.0;
		real m_rect_rounding = 2.0;

		bool m_createIsoTriState = false;
		real m_iso_tri_b = 3.0;
		real m_iso_tri_h = 3.0;
		real m_iso_tri_inner_radius = 0.0;
		real m_iso_tri_rounding = 2.0;

		// imgui stuff
		bool m_renderGUI = true;

		// put node stuff here later
		std::string m_nodeText = "x:\nv:\n";
		std::string m_pointsViewStr = "";
	};



}