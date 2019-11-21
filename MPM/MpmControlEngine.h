#pragma once
#include "Constants.h"
#include "EngineHeader.h"


#include "Shader.h"
#include "Shape.h"
#include "PointCloud.h"
#include "Grid.h"

#include "OpenGLScreen.h"

#include "MpmEngine.h"
#include "MpmAlgorithmEngine.h"
#include "EnergyFunctions.h"

#include "MpmSpaceTimeControl.h"

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
		MpmControlEngine() {
			InitShaders(); 
			InitSTCG(); 
		}
		~MpmControlEngine() { CleanupShaders(); }

		//void Render(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen);
		void GUI();

		// for ImGui menu bar
		void Menu();

		void Render(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen);


		// for mpm engine
		/*void ProcessMouseInput();
		void ProcessKeyboardInput(GLFWwindow* window, real lag);*/
		void SetMpmEngine(MpmEngine* mpmEngine) {
			m_mpmEngine = mpmEngine;
		}
		void SetAlgorithmEngine(std::shared_ptr<MpmAlgorithmEngine> mpmAlgorithmEngine) {
			m_mpmAlgorithmEngine = mpmAlgorithmEngine;
		}
		void SetMpmGeometryEngine(std::shared_ptr<MpmGeometryEngine> mpmGeometryEngine) {
			m_mpmGeometryEngine = mpmGeometryEngine;
		}
		
	private:

		// Other Engines
		MpmEngine* m_mpmEngine = nullptr;
		std::shared_ptr<MpmAlgorithmEngine> m_mpmAlgorithmEngine = nullptr;
		std::shared_ptr<MpmGeometryEngine> m_mpmGeometryEngine = nullptr;

		void InitShaders();
		void CleanupShaders();

		void ImGuiExternalForceController();
		void ImGuiDeformationGradientController();
		void ImGuiMaterialParameterController();
		void ImGuiControlPointViewer();

		// control
		bool m_imguiExternalForceController = false;
		bool m_imguiDeformationGradientController = false;
		bool m_imguiMaterialParameterController = false;
		bool m_imguiControlPointViewer = false;

		bool m_animateSimStates = false;
		bool m_animateLoop = false;
		int m_currSimState = 0;
		int m_simStateFramesPerFrame = 2;

		// INTERACTIONS
		std::unique_ptr<ComputeShader> m_pSetDeformationGradients = nullptr;
		std::unique_ptr<ComputeShader> m_pMultDeformationGradients = nullptr;
		std::unique_ptr<ComputeShader> m_pSetLameParamters = nullptr;

		// GRAPHICS
		std::unique_ptr<StandardShader> m_pRenderControlPointCloud = nullptr;

		void RenderControlPointCloud(vec2 zoomPoint, real zoomFactor, std::shared_ptr<control::ControlPointCloud> pointCloud, GLuint ssbo);

		/******************** EXTERNAL FORCE CONTROLLER ********************/
	public:
		real m_drag = 0.5;
		vec2 m_globalForce = vec2(0.0, -9.81);
		GLreal m_mousePower = 25.0;
	private:


		/******************** INTERNAL FORCE CONTROLLER ********************/
		void SetDeformationGradientsGLSL(std::string pointCloudID, mat2 Fe, mat2 Fp, bool setSelected);
		void SetDeformationGradientsGLSL(std::shared_ptr<PointCloud> pointCloud, mat2 Fe, mat2 Fp, bool setSelected);
		void MultiplyDeformationGradientsGLSL(std::string pointCloudID, mat2 multFe, mat2 multFp, bool setSelected);
		void SetLameParametersGLSL(std::string pointCloudID, real lam, real mew, bool setSelected);



		/******************** DEFORMATION GRADIENT SPACETIME CONTROL ********************/
		/*
		

		std::vector<MaterialPoint> m_controlPointCloudOriginalPoints;
		
		void SaveControlPointCloudOriginalPoints(std::shared_ptr<PointCloud> pointCloud);
		void ResetControlPointCloudPointsToSaved(std::shared_ptr<PointCloud> pointCloud);
		*/
		
		

		bool m_renderOutputPointCloud = true;
		bool m_renderControlPointCloud = true;
		bool m_renderTargetPointCloud = false;


		void ImGuiDeformationGradientSpaceTimeController();
		bool m_imguiDeformationGradientSpaceTimeController = false;


		// Using new control framework:
		//void CreateControlPointCloud(std::shared_ptr<control::ControlPointCloud> controlPointCloud, std::shared_ptr<const PointCloud> pointCloud);
		/*std::shared_ptr<control::ControlPointCloud> m_targetPointCloud = nullptr;
		std::shared_ptr<control::ControlPointCloud> m_controlPointCloud = nullptr;
		std::shared_ptr<control::ControlGrid> m_controlGrid = nullptr;*/
		std::shared_ptr<control::MPMSpaceTimeComputationGraph> m_stcg = nullptr;
		void InitSTCG();
	};

}