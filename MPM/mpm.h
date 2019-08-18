#pragma once
#include "Constants.h"
#include "Shader.h"
#include "Shape.h"
#include "sdf.h"
#include "PointCloud.h"
#include "Grid.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <chrono>
#include <functional>

namespace mpm {

	class MpmManager {

	public:
		MpmManager() {}
		~MpmManager() { CleanupComputeShaderPipeline();  };

		bool InitComputeShaderPipeline();
		bool CleanupComputeShaderPipeline();

		void MpmTimeStep(float dt);

		void Render();

		void RenderGUI();
		void UpdateNodeData();

		void SetGlobalForce(glm::vec2 _globalForce) {
			m_globalForce = _globalForce;
		}

		void SetMouseValues(glm::vec2 _mousePos, bool _leftButtonDown, bool _rightButtonDown) {
			m_mousePos = _mousePos;
			m_leftButtonDown = _leftButtonDown;
			m_rightButtonDown = _rightButtonDown;
		}


		std::shared_ptr<StandardShader> GetMouseShader() { return m_mouseShader; }

	private:

		

		PointCloud GenPointCloud(const Shape& shape, sdf::sdFunc _sdf,
			const float gridDimX, const float gridDimY, const float particleSpacing, 
			const float density, const float youngMod, const float poisson,
			glm::vec2 initialVelocity);

		void CreateDemo();
		void CalculatePointCloudVolumes();

		std::unique_ptr<ComputeShader> m_gReset = nullptr;
		std::unique_ptr<ComputeShader> m_p2gScatter = nullptr;
		std::unique_ptr<ComputeShader> m_p2gGather = nullptr;
		std::unique_ptr<ComputeShader> m_gUpdate = nullptr;
		std::unique_ptr<ComputeShader> m_g2pGather = nullptr;
		

		std::unique_ptr<ComputeShader> m_p2gCalcVolumes = nullptr;
		std::unique_ptr<ComputeShader> m_g2pCalcVolumes = nullptr;

		std::unique_ptr<StandardShader> m_pPointCloud = nullptr;

		std::shared_ptr<StandardShader> m_mouseShader = nullptr;

		size_t m_numPointClouds = 0;
		GLuint pointCloudSSBO[20];
		GLuint gridSSBO;
		GLuint VisualizeVAO;

		std::vector<PointCloud> m_pointClouds;
		Grid m_grid;

		glm::vec2 m_globalForce = glm::vec2(0.f);
		GLfloat m_globalForceArray[2] = { 0.f, 0.f }; // for input with imgui
		float m_set_dt = 1.f / 60.f;
		float m_dt = 1.f / 60.f;
		bool m_paused = false;

		glm::vec2 m_mousePos = glm::vec2(0.0, 0.0);
		bool m_leftButtonDown = false;
		bool m_rightButtonDown = false;
		GLfloat m_mousePower = 25.f;
		
		GLfloat m_circle_x[2] = { 35.f, 10.f };
		GLfloat m_circle_v[2] = { -3.f, 0.f };
		
		GLfloat m_donut_x[2] = { 15.f, 10.f };
		GLfloat m_donut_v[2] = { 3.f, 0.f };
		glm::vec2 m_cx = glm::vec2(30.f, 10.f);
		int m_pointCloudSelect = 0;
		int m_timeStep = 0;
		float m_time = 0.f;

		// imgui stuff
		bool m_renderGUI = true;
		int m_node[2] = { 26, 8 };
		std::string m_nodeText = "x:\nv:\n";
		std::string m_pointsViewStr = "";
	};



}