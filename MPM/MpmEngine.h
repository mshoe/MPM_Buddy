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
		

		void ProcessKeyboardInput(GLFWwindow* window, real lag);
		void ProcessMouseInput(GLFWwindow* window, real lag);
		void HandleStates();


	private:

		int m_chunks_x = 4;
		int m_chunks_y = 4;

		//*** MPM FUNCTIONS ***//
		void MpmReset();
		void MpmTimeStep(real dt);
		void MpmTimeStepP2G(real dt);
		void MpmTimeStepExplicitGridUpdate(real dt);
		void MpmTimeStepG2P(real dt);
		void MpmTimeStepSemiImplicitCRGridUpdate(real dt);
		void MpmCRInit(real dt);
		bool MpmCRStep(real dt, real& L2_norm_rk, bool& L2_converged, bool& L_inf_converged);
		void MpmCREnd(real dt);
		void MpmTimeStepSemiImplicitDirectSolve(real dt);
		void CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);

		//*** RENDERING FUNCTIONS ***//
	public:
		void Render();
	private:
		void RenderScreenShader(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen);
		void RenderPointClouds(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pointShader);
		void RenderGrid(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader);
		void RenderMarchingSquares(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader);
		void RenderPolygon(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> polygonShader);
		void RenderPWLine(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pwLineShader);
		

		//*** INTERACTIVE FUNCTIONS ***//
		void SetDeformationGradients(std::string pointCloudID, mat2 Fe, mat2 Fp);
		mat2 m_setFe = mat2(1.0);
		mat2 m_setFp = mat2(1.0);
		void MultiplyDeformationGradients(std::string pointCloudID, mat2 multFe, mat2 multFp);
		std::vector<mat2> m_multFeVector;
		mat2 m_multFe = mat2(1.0);
		mat2 m_multFp = mat2(1.0);

		//*** GUI FUNCTIONS ***//
	public:
		void RenderGUI();
	private:
		void RenderWindowManager();
		void RenderTimeIntegrator();
		void RenderForceController();
		void RenderGeometryEditor();
		void RenderMaterialParametersEditor();
		void RenderGridNodeViewer();
		void RenderMaterialPointViewer();
		void RenderZoomWindow();

		bool m_renderTimeIntegrator = true;
		bool m_renderForceController = true;
		bool m_renderMaterialParametersEditor = true;
		bool m_renderGeometryEditor = true;
		bool m_renderGridNodeViewer = true;
		bool m_renderMaterialPointViewer = true;
		bool m_renderZoomWindow = true;

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
		bool m_fixedPointCloud = false;
		bool m_invertedSdf = false;
		std::shared_ptr<PointCloud> GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
			const real gridDimX, const real gridDimY,
			const real inner_rounding, const real outer_rounding, 
			const MaterialParameters &parameters,
			const GLuint comodel, sdf::SDF_OPTION sdfOption, 
			bool inverted, bool fixed,
			vec2 initialVelocity, glm::highp_fvec4 color);
		void ClearCreateStates() {
			m_createCircleState = false;
			m_createRectState = false;
			m_createIsoTriState = false;
		}

		std::shared_ptr<sdf::Polygon> m_polygon = nullptr;
		bool m_addPolygonVertexState = false;
		void GenPointCloudPolygon();
		int m_polygonCount = 0;
		bool m_renderPolygon = false;

		std::shared_ptr<sdf::PWLine> m_pwLine = nullptr;
		bool m_addPWLineVertexState = false;
		real m_pwLineRounding = 2.0;
		void GenPointCloudPWLine();
		int m_pwLineCount = 0;
		bool m_renderPWLine = false;
		//void GenPointCloudLineDivider();
		////std::vector<>
		//real m_line_m = 2.0;
		//real m_line_b = -50.0;
		//real m_line2_m = -2.0;
		//real m_line2_b = 178.0;
		


		void PrintGridData();


		//*** SHADERS ***//
		std::unique_ptr<ComputeShader> m_gReset = nullptr;
		std::unique_ptr<ComputeShader> m_p2gScatter = nullptr;
		std::unique_ptr<ComputeShader> m_p2gGather = nullptr;
		std::unique_ptr<ComputeShader> m_gUpdate = nullptr;
		std::unique_ptr<ComputeShader> m_g2pGather = nullptr;
		

		std::unique_ptr<ComputeShader> m_p2gCalcVolumes = nullptr;
		std::unique_ptr<ComputeShader> m_g2pCalcVolumes = nullptr;

		// INTERACTIONS
		std::unique_ptr<ComputeShader> m_pSetDeformationGradients = nullptr;
		std::unique_ptr<ComputeShader> m_pMultDeformationGradients = nullptr;

		// RENDERING
		std::shared_ptr<StandardShader> m_pPointCloudShader = nullptr;
		std::shared_ptr<StandardShader> m_pPointCloudVectorShader = nullptr;
		std::shared_ptr<StandardShader> m_mouseShader = nullptr;
		std::shared_ptr<StandardShader> m_zoomWindowShader = nullptr;
		std::shared_ptr<StandardShader> m_gridShader = nullptr;
		std::shared_ptr<StandardShader> m_gridShaderVector = nullptr;
		std::shared_ptr<StandardShader> m_gridShaderMarchingSquares = nullptr;

		std::shared_ptr<StandardShader> m_borderShader = nullptr;
		std::shared_ptr<StandardShader> m_polygonShader = nullptr;
		std::shared_ptr<StandardShader> m_pwLineShader = nullptr;



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
		double m_maxSpeedClamp = 25.0;
		double m_minSpeedClamp = 0.0;
		bool m_visualizeSpeed = true;
		double m_maxEnergyClamp = 100.0;
		double m_minEnergyClamp = 0.0;
		bool m_visualizeEnergy = true;
		bool m_viewPointClouds = true;
		
		// GRID VISUALIZATION STUFF
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
		int m_gridVectorOption = 2; // VELOCITY = 0, ACCELERATION = 1, FORCE = 2, RESIDUAL = 3
		int m_gridPointSizeScalingOption = 0;
		real m_maxGridVectorLength = 25.0;
		real m_maxGridVectorVisualLength = 5.0;
		bool m_viewMarchingSquares = false;
		double m_isoMass = 5.0;
		glm::highp_fvec4 m_marchingSquaresColor = glm::highp_fvec4(1.0f);
		GridNode m_gn; // selecting grid node
		bool m_selectNodeState = false;
		void UpdateNodeData();
		void SelectNodesInShape(sdf::Shape& shape,
			const int gridDimX, const int gridDimY,
			const real inner_rounding, const real outer_rounding, 
			sdf::SDF_OPTION sdfOption,
			bool inverted);
		bool m_collectiveNodeSelectionGraphics = false;
		void ClearNodesSelected(const int gridDimX, const int gridDimY);
		void CalculateNodalAccelerations(const int gridDimX, const int gridDimY, real accStr);
		void ClearNodalAcclerations(const int gridDimX, const int gridDimY);

		real m_drag = 0.5;
		vec2 m_globalForce = vec2(0.0, -9.81);
		//real m_set_dt = 1.0 / 120.0;
		real m_dt = 1.0 / 120.0;
		bool m_paused = true;

		// SEMI-IMPLICIT CR
		bool m_semi_implicit_CR = false;
		real m_semi_implicit_ratio = 1.0;
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
		bool m_rt = true; // realtime

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

		real m_lam = 38888.9;
		real m_mew = 58333.0;


		GLuint m_comodel = FIXED_COROTATIONAL_ELASTICITY;
		void ChangeMaterialParameters(GLuint);
		

		float m_backgroundColor[4] = { 0.0f, 0.0f, 0.0f, 1.0f };

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