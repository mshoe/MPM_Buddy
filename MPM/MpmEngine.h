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

		std::shared_ptr<StandardShader> GetNodeShader() {
			return m_nodeShader;
		}
		bool GetNodeShaderActive() {
			return m_nodeGraphicsActive;
		}

	private:

		//*** FUNCTIONS ***//
		void MpmTimeStep(float dt);
		void MpmImplictTimeCR(float dt);

		void UpdateNodeData();
		void SetGlobalForce(glm::vec2 _globalForce) {
			m_globalForce = _globalForce;
		}

		std::shared_ptr<PointCloud> GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
			const float gridDimX, const float gridDimY, 
			const float particleSpacing, const float density, 
			const float inner_rounding, const float outer_rounding, 
			const float youngMod, const float poisson,
			const float crit_c, const float crit_s, const float hardening,
			const GLuint comodel,
			glm::vec2 initialVelocity, glm::vec4 color);
		void CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);

		void CreateDemo();
		
		void PrintGridData();


		//*** SHADERS ***//
		std::unique_ptr<ComputeShader> m_gReset = nullptr;
		std::unique_ptr<ComputeShader> m_p2gScatter = nullptr;
		std::unique_ptr<ComputeShader> m_p2gGather = nullptr;
		std::unique_ptr<ComputeShader> m_gUpdate = nullptr;
		std::unique_ptr<ComputeShader> m_g2pGather = nullptr;
		

		std::unique_ptr<ComputeShader> m_p2gCalcVolumes = nullptr;
		std::unique_ptr<ComputeShader> m_g2pCalcVolumes = nullptr;

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

		float m_drag = 0.5f;
		glm::vec2 m_globalForce = glm::vec2(0.f);
		GLfloat m_globalForceArray[2] = { 0.f, 0.f }; // for input with imgui
		//float m_set_dt = 1.f / 120.f;
		float m_dt = 1.f / 120.f;
		bool m_paused = true;
		bool m_implicit = false;
		float m_implicit_ratio = 1.f;
		int m_max_conj_res_iter = 30;

		//int m_pointCloudSelect = 0;
		int m_timeStep = 0;
		float m_time = 0.f;
		bool m_rt = true;

		GLfloat m_mousePower = 25.f;


		// Reorganize this stuff
		unsigned int m_circleCount = 0;
		unsigned int m_rectCount = 0;
		unsigned int m_isoTriCount = 0;

		//std::vector<PointCloud> m_pointClouds;
		std::unordered_map<std::string, std::shared_ptr<PointCloud>> m_pointCloudMap;
		std::vector<char> m_pointCloudSelect;
		
		float m_youngMod = 400.f;
		float m_poisson = 0.3f;

		float m_particleSpacing = 0.25f;
		float m_density = 0.16f;
		
		float m_crit_c = 0.025f;
		float m_crit_s = 0.0075f;
		float m_hardening = 10.f;

		GLuint m_comodel = 1;

		float m_color[4] = { 1.f, 0.f, 0.f, 1.0f};
		
		bool m_createCircleState = false;
		float m_circle_r = 5.f;
		float m_circle_inner_radius = 0.f;
		float m_circle_rounding = 0.f;

		bool m_createRectState = false;
		float m_rect_b = 3.f;
		float m_rect_h = 3.f;
		float m_rect_inner_radius = 0.f;
		float m_rect_rounding = 2.f;

		bool m_createIsoTriState = false;
		float m_iso_tri_b = 3.f;
		float m_iso_tri_h = 3.f;
		float m_iso_tri_inner_radius = 0.f;
		float m_iso_tri_rounding = 2.f;

		// imgui stuff
		bool m_renderGUI = true;

		// put node stuff here later
		std::string m_nodeText = "x:\nv:\n";
		std::string m_pointsViewStr = "";
	};



}