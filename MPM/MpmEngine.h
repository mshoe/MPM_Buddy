#pragma once
#include "Engine.h"

#include "Constants.h"
#include "Shader.h"
#include "Shape.h"
#include "PointCloud.h"
#include "Grid.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <tuple>

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

	private:

		//*** FUNCTIONS ***//
		void MpmTimeStep(float dt);

		void UpdateNodeData();
		void SetGlobalForce(glm::vec2 _globalForce) {
			m_globalForce = _globalForce;
		}

		std::shared_ptr<PointCloud> GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
			const float gridDimX, const float gridDimY, const float particleSpacing, 
			const float density, const float youngMod, const float poisson,
			glm::vec2 initialVelocity, glm::vec3 color);
		void CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);

		//void CreateDemo();
		


		//*** SHADERS ***//
		std::unique_ptr<ComputeShader> m_gReset = nullptr;
		std::unique_ptr<ComputeShader> m_p2gScatter = nullptr;
		std::unique_ptr<ComputeShader> m_p2gGather = nullptr;
		std::unique_ptr<ComputeShader> m_gUpdate = nullptr;
		std::unique_ptr<ComputeShader> m_g2pGather = nullptr;
		

		std::unique_ptr<ComputeShader> m_p2gCalcVolumes = nullptr;
		std::unique_ptr<ComputeShader> m_g2pCalcVolumes = nullptr;

		std::unique_ptr<StandardShader> m_pPointCloudShader = nullptr;

		
		
		GLuint gridSSBO;
		GLuint VisualizeVAO;

		
		Grid m_grid;

		glm::vec2 m_globalForce = glm::vec2(0.f);
		GLfloat m_globalForceArray[2] = { 0.f, 0.f }; // for input with imgui
		float m_set_dt = 1.f / 60.f;
		float m_dt = 1.f / 60.f;
		bool m_paused = false;

		int m_pointCloudSelect = 0;
		int m_timeStep = 0;
		float m_time = 0.f;

		GLfloat m_mousePower = 25.f;


		// Reorganize this stuff
		unsigned int m_circleCount = 0;
		//std::vector<PointCloud> m_pointClouds;
		std::unordered_map<std::string, std::shared_ptr<PointCloud>> m_pointCloudMap;
		//GLuint pointCloudSSBO[20];
		/*GLfloat m_circle_x[2] = { 35.f, 10.f };
		GLfloat m_circle_v[2] = { -3.f, 0.f };
		
		GLfloat m_donut_x[2] = { 15.f, 10.f };
		GLfloat m_donut_v[2] = { 3.f, 0.f };
		glm::vec2 m_cx = glm::vec2(30.f, 10.f);*/
		


		bool m_createCircleState = false;

		// imgui stuff
		bool m_renderGUI = true;
		int m_node[2] = { 26, 8 };
		std::string m_nodeText = "x:\nv:\n";
		std::string m_pointsViewStr = "";
	};



}