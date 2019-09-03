#pragma once

#include "Constants.h"

#include "Engine.h"

#include "Shader.h"
#include "Shape.h"
#include "PointCloud.h"
#include "Grid.h"

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

		void Render();
		void RenderGUI();

		void HandleInput();

		//void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

		void SetPausedState(bool state) { m_paused = state; }
		bool GetPausedState() { return m_paused; }

		void SetCreateCircleState(bool state) {
			if (state) {
				m_createRectState = false;
				m_createIsoTriState = false;
			}
			m_createCircleState = state; 
		}


		void SetCreateRectState(bool state) {
			if (state) {
				m_createCircleState = false;
				m_createIsoTriState = false;
			}
			m_createRectState = state;
		}

		void SetCreateIsoTriState(bool state) {
			if (state) {
				m_createCircleState = false;
				m_createRectState = false;
			}
			m_createIsoTriState = state;
		}


		// temporarily putting node stuff here
		bool m_nodeGraphicsActive = false;
		std::shared_ptr<StandardShader> m_nodeShader = nullptr;
		int m_node[2] = { 26, 8 };

		// selecting grid node
		GridNode m_gn;
		bool m_selectNodeState = false;
		void UpdateNodeData();

		std::shared_ptr<StandardShader> GetNodeShader() {
			return m_nodeShader;
		}
		bool GetNodeShaderActive() {
			return m_nodeGraphicsActive;
		}

	private:

		//*** MPM FUNCTIONS ***//
		void MpmTimeStep(real dt);
		void MpmTimeStepP2G(real dt);
		void MpmTimeStepExplicitGridUpdate(real dt);
		void MpmTimeStepSemiImplicitGridUpdate(real dt);
		void MpmTimeStepG2P(real dt);
		void MpmCRInit(real dt);
		bool MpmCRStep(real dt);
		void MpmCREnd(real dt);
		void CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);
		void SetReferenceConfig(std::string pointCloudID);

		//*** GUI FUNCTIONS ***//
		void RenderTimeIntegrator();
		void RenderForceController();
		void RenderGeometryEditor();
		void RenderGridNodeViewer();
		void RenderMaterialPointViewer();

		//*** GEOMETRY EDITOR FUNCTIONS ***//
		std::shared_ptr<PointCloud> GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
			const real gridDimX, const real gridDimY,
			const real inner_rounding, const real outer_rounding, 
			const MaterialParameters &parameters,
			const GLuint comodel,
			vec2 initialVelocity, glm::highp_fvec4 color);

		
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

		std::unique_ptr<StandardShader> m_pPointCloudShader = nullptr;

		

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

		
		Grid m_grid;

		real m_drag = 0.5;
		vec2 m_globalForce = vec2(0.0);
		//real m_set_dt = 1.0 / 120.0;
		real m_dt = 1.0 / 120.0;
		bool m_paused = true;
		bool m_implicit = false;
		real m_implicit_ratio = 1.0;
		int m_max_conj_res_iter = 30;// GRID_SIZE_X* GRID_SIZE_Y;
		int m_cr_step = 0;

		//int m_pointCloudSelect = 0;
		int m_timeStep = 0;
		real m_time = 0.0;
		bool m_rt = true;

		GLreal m_mousePower = 25.0;


		// Reorganize this stuff
		unsigned int m_circleCount = 0;
		unsigned int m_rectCount = 0;
		unsigned int m_isoTriCount = 0;

		//std::vector<PointCloud> m_pointClouds;
		std::unordered_map<std::string, std::shared_ptr<PointCloud>> m_pointCloudMap;
		std::vector<char> m_pointCloudSelect;



		// selecting material points
		MaterialPoint m_mp;

		double m_maxEnergyClamp = 100.0;
		double m_minEnergyClamp = 0.0;
		bool m_visualizeEnergy = false;
		
		MaterialParameters m_mpParameters;
		MaterialParameters m_fixedCorotatedParameters;
		MaterialParameters m_simpleSnowParameters;


		GLuint m_comodel = FIXED_COROTATIONAL_ELASTICITY;
		void ChangeMaterialParameters(GLuint);
		

		float m_color[4] = { 1.0f, 0.0f, 0.0f, 1.0f}; // color needs to be float
		
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