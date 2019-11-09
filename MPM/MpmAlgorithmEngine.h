#pragma once

#include "Constants.h"
#include "EngineHeader.h"

#include "Shader.h"
#include "Shape.h"
#include "PointCloud.h"
#include "Grid.h"

#include "OpenGLScreen.h"

#include "MpmEngine.h"
#include "MpmControlEngine.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <map>
#include <tuple>

namespace mpm {
	class MpmAlgorithmEngine {
	public:
		MpmAlgorithmEngine() { InitShaders(); }
		~MpmAlgorithmEngine() { CleanupShaders(); }

		//void Render(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen);
		void GUI();




		// for ImGui menu bar
		void Menu();

		// for mpm engine
		void ProcessMouseInput();
		void ProcessKeyboardInput(GLFWwindow* window, real lag);
		void SetMpmEngine(MpmEngine* mpmEngine) {
			m_mpmEngine = mpmEngine;
		}
		void SetMpmControlEngine(std::shared_ptr<MpmControlEngine> mpmControlEngine) {
			m_mpmControlEngine = mpmControlEngine;
		}
		void SetMpmGeometryEngine(std::shared_ptr<MpmGeometryEngine> mpmGeometryEngine) {
			m_mpmGeometryEngine = mpmGeometryEngine;
		}

		bool m_paused = true;

		void Update();
		
		// geometry engine wants access to this
		void CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);

		/******************** TIME INTEGRATOR ********************/
		int m_timeStep = 0;
		real m_time = 0.0;
		real m_dt = 1.0 / 120.0;
		bool m_rt = true; // realtime
	private:

		// Other Engines
		MpmEngine* m_mpmEngine = nullptr;
		std::shared_ptr<MpmControlEngine> m_mpmControlEngine = nullptr;
		std::shared_ptr<MpmGeometryEngine> m_mpmGeometryEngine = nullptr;

		void InitShaders();
		void CleanupShaders();

		enum class MPM_ALGORITHM_CODE {
			GLSL = 0,
			CPP = 1
		};

		MPM_ALGORITHM_CODE m_algo_code = MPM_ALGORITHM_CODE::GLSL;

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



		void ImGuiMaterialParametersEditor();
		void ImGuiCPUMode();


		bool m_imguiMaterialParametersEditor = false;
		bool m_imguiCPUMode = false;

		bool m_semiImplicitCPP = false;
		double m_beta = 1.0;

		void GetPointCloudVolumesFromGPUtoCPU(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);

		void MapCPUPointCloudsToGPU();
		void MapCPUGridToGPU();

		void ImGuiTimeIntegrator();
		bool m_imguiTimeIntegrator = false;

		/******************** MPM ALGORITHM COMPUTE SHADERS ********************/
		std::unique_ptr<ComputeShader> m_gReset = nullptr;
		std::unique_ptr<ComputeShader> m_p2gScatter = nullptr;
		std::unique_ptr<ComputeShader> m_p2gGather = nullptr;
		std::unique_ptr<ComputeShader> m_gUpdate = nullptr;
		std::unique_ptr<ComputeShader> m_g2pGather = nullptr;


		std::unique_ptr<ComputeShader> m_p2gCalcVolumes = nullptr;
		std::unique_ptr<ComputeShader> m_g2pCalcVolumes = nullptr;

		// Implict time integration shaders
		std::unique_ptr<ComputeShader> m_p2g2pDeltaForce = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsInitPart1 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsInitPart2 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsInitPart3 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsStepPart1 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsStepPart2 = nullptr;
		std::unique_ptr<ComputeShader> m_gConjugateResidualsConclusion = nullptr;

		

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

		

		// TRANSFER SCHEMES (PIC/RPIC/APIC)
		enum class TRANSFER_SCHEME {
			PIC = 0,
			RPIC = 1,
			APIC = 2,
			MLS = 3
		};
		std::vector<std::string> m_transferSchemeStrVec = {
			"PIC",
			"RPIC (not working)",
			"APIC",
			"MLS"
		};
		TRANSFER_SCHEME m_transferScheme = TRANSFER_SCHEME::APIC;

	public:


		/******************** MATERIAL PARAMETERS EDITOR ********************/
		MaterialParameters m_mpParameters;
		std::vector<MaterialParameters> m_energyModels = std::vector<MaterialParameters>(size_t(ENERGY_MODEL::Count), MaterialParameters());
		std::vector<std::string> m_energyModelsStrVec = {
			"Neohookean Elasticity",
			"Fixed Corotational Elasticity",
			"Snow (Stomakhin et. al 2013)"
		};
		ENERGY_MODEL m_comodel = ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY;
		void ChangeEnergyModel(ENERGY_MODEL);
		

	};
}