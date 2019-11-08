#pragma once
#include "Constants.h"
#include "EngineHeader.h"


#include "Shader.h"
#include "Shape.h"
#include "PointCloud.h"
#include "Grid.h"

#include "OpenGLScreen.h"

#include "MpmEngine.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <map>
#include <tuple>

namespace mpm {
	class MpmControlEngine {
	public:
		MpmControlEngine() { InitShaders(); }
		~MpmControlEngine() { CleanupShaders(); }

		//void Render(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen);
		void GUI();

		// for ImGui menu bar
		void Menu();

		// for mpm engine
		/*void ProcessMouseInput();
		void ProcessKeyboardInput(GLFWwindow* window, real lag);*/
		void SetMpmEngine(MpmEngine* mpmEngine) {
			m_mpmEngine = mpmEngine;
		}
	private:

		// Other Engines
		MpmEngine* m_mpmEngine = nullptr;

		void InitShaders();
		void CleanupShaders();

		void ImGuiExternalForceController();
		void ImGuiDeformationGradientController();
		void ImGuiMaterialParameterController();

		// control
		bool m_imguiExternalForceController = false;
		bool m_imguiDeformationGradientController = false;
		bool m_imguiMaterialParameterController = false;

		// INTERACTIONS
		std::unique_ptr<ComputeShader> m_pSetDeformationGradients = nullptr;
		std::unique_ptr<ComputeShader> m_pMultDeformationGradients = nullptr;
		std::unique_ptr<ComputeShader> m_pSetLameParamters = nullptr;

		/******************** EXTERNAL FORCE CONTROLLER ********************/
	public:
		real m_drag = 0.5;
		vec2 m_globalForce = vec2(0.0, -9.81);
		GLreal m_mousePower = 25.0;
	private:


		/******************** INTERNAL FORCE CONTROLLER ********************/
		void SetDeformationGradients(std::string pointCloudID, mat2 Fe, mat2 Fp, bool multSelected);
		void MultiplyDeformationGradients(std::string pointCloudID, mat2 multFe, mat2 multFp, bool setSelected);
		void SetLameParameters(std::string pointCloudID, real lam, real mew, bool setSelected);
	};

}