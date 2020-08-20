#pragma once

#include "Constants.h"
#include "EngineHeader.h"

#include "Shader.h"
#include "Shape.h"
#include "PointCloud.h"
#include "Grid.h"

#include "ShapeFunctions.h"

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

	


	// for sparse matrix vis with opengl
	struct EigenTriplet {
		glm::ivec2 index;
		real val;
	};

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
		void CalculatePointCloudVolumesCPP(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);

		/******************** TIME INTEGRATOR ********************/
		int m_timeStep = 0;
		real m_time = 0.0;
		real m_dt = 1.0 / 120.0;
		bool m_rt = true; // realtime


		// allow even smaller grid sizes for CPU mode
		int m_cppChunkX = 32;
		int m_cppChunkY = 32;

		enum class MPM_ALGORITHM_CODE {
			GLSL = 0,
			CPP = 1
		};

		volatile MPM_ALGORITHM_CODE m_algo_code = MPM_ALGORITHM_CODE::CPP;


		// MPM ALGORITHMS
		enum class MPM_ALGO {
			MLS = 0,
			MUSL = 1,
			USF = 2,
			USL = 3,
			SE = 4
		};
		std::vector<std::string> m_mpmAlgoStrVec = {
			"MLS",
			"MUSL",
			"USF",
			"USL",
			"Symplectic Euler"
		};
		volatile MPM_ALGO m_mpm_algo = MPM_ALGO::MUSL;

		// BASIS FUNCTIONS
		// enum declared in ShapeFunctions.h
		std::vector<std::string> m_basisFunctionStrVec = {
			"Linear",
			"Quadratic B-Spline",
			"Cubic B-Spline"
		};
		Basis::BasisType m_basisFunction = Basis::BasisType::CUBIC_B_SPLINE;


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

	private:

		// Other Engines
		MpmEngine* m_mpmEngine = nullptr;
		std::shared_ptr<MpmControlEngine> m_mpmControlEngine = nullptr;
		std::shared_ptr<MpmGeometryEngine> m_mpmGeometryEngine = nullptr;

		void InitShaders();
		void CleanupShaders();

		
		// Piola-Kirchoff stress calculation
		void MpmTimeStepP_Stress(real dt);


		/******************** MLS MPM GLSL COMPUTE SHADER IMPLEMENTATION ********************/
	public:
		void MpmReset_GLSL();
		void MpmTimeStep_GLSL(real dt);
	private:
		void MpmTimeStepP2G_GLSL(real dt);
		void MpmTimeStepExplicitGridUpdate_GLSL(real dt);
		void MpmTimeStepG2P_GLSL(real dt);
		void MpmTimeStepSemiImplicitCRGridUpdate_GLSL(real dt);
		void MpmCRInit_GLSL(real dt);
		bool MpmCRStep_GLSL(real dt, real& L2_norm_rk, bool& L2_converged, bool& L_inf_converged);
		void MpmCREnd_GLSL(real dt);
		void CalculatePointCloudVolumes_GLSL(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);

		/******************** MLS MPM CPU C++ IMPLEMENTATION ********************/
	public:
		void RunMPMSimulationMLS(real dt, size_t num_steps, bool debugOutput, bool renderAfter);
		void MpmReset_MLS();
		void MpmTimeStep_MLS(real dt);
	private:

		void MassMomentumToNode_MLS(const mpm::MaterialPoint& mp, mpm::GridNode& node, real dt);
		void UpdateNodeVelocity_MLS(mpm::GridNode& node, real dt);
		void VelocityToParticle_MLS(const mpm::GridNode& node, mpm::MaterialPoint& mp, real dt);


		void MpmTimeStepP2G_MLS(real dt);
		void MpmTimeStepExplicitGridUpdate_MLS(real dt);
		void MpmTimeStepSemiImplicitGridUpdate_MLS(real dt, real beta);
		void MpmTimeStepG2P_MLS(real dt);
		void MpmTimeStepP2_MLS(real dt);

		

		bool m_USL = true;

		void MpmTimeStep(real dt);
		void MpmTimeStepAlgoSelector(real dt);

		/******************** MUSL MPM CPU C++ IMPLEMENTATION ********************/
	public:
		void MpmTimeStep_MUSL(real dt);
	private:
		void MassToNode_MUSL(const mpm::MaterialPoint& mp, mpm::GridNode& node);
		void UpdateNodeMomentum_MUSL(mpm::GridNode& node, vec2 f_ext, real dt);
		void NodeToParticleVelocity_MUSL(const mpm::GridNode& node, mpm::MaterialPoint& mp, real dt);
		void VelocityToNode_MUSL(const mpm::MaterialPoint& mp, mpm::GridNode& node);
		void CalculateNodeVelocity_MUSL(mpm::GridNode& node, double dt);
		void NodeToParticlePosition_MUSL(const mpm::GridNode& node, mpm::MaterialPoint& mp, mat2& Lp, double dt);
		void ParticleUpdateStressStrain_MUSL(mpm::MaterialPoint& mp, real dt, ENERGY_MODEL comodel);
		

		void MpmTimeStepP2G_MUSL(real dt);
		void MpmTimeStepG_Momentum_MUSL(real dt);
		void MpmTimeStepG2P_Velocity_MUSL(real dt);
		void MpmTimeStepP2G_Velocity_MUSL(real dt);
		void MpmTimeStepG_Velocity_MUSL(real dt);
		void MpmTimeStepG2P_Position_MUSL(real dt);

		/******************** USF MPM CPU C++ IMPLEMENTATION ********************/
	public:
		void MpmTimeStep_USF(real dt);

	private:
		void MpmTimeStepP2G_Velocity_USF(real dt);
		void MpmTimeStepG_Velocity_USF(real dt);
		void MpmTimeStepG2P_GradientVelocity_USF(real dt);
		void MpmTimeStepP2G_Forces_USF(real dt);
		void MpmTimeStepG_Momentum_USF(real dt);
		void MpmTimeStepG2P_PositionVelocity_USF(real dt);


		/******************** USL MPM CPU C++ IMPLEMENTATION ********************/
	public:
		void MpmTimeStep_USL(real dt);

	private:

		/******************** Symplectic Euler MPM CPU C++ IMPLEMENTATION ********************/
	public:
		void MpmTimeStep_SE(real dt);
	private:
		void MpmTimeStepP2G_SE(real dt);
		void MpmTimeStepG_Velocity_SE(real dt);
		void MpmTimeStepG2P_SE(real dt);


		Basis::NodeGetter nodeGetter;
		
		

		




		void ImGuiMaterialParametersEditor();
		void ImGuiCPUMode();


		bool m_imguiMaterialParametersEditor = false;
		bool m_imguiCPUMode = false;

		bool m_semiImplicitCPP = false;
		double m_beta = 1.0;

		void GetPointCloudVolumesFromGPUtoCPU(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud);

		

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

		// graphics

		// Sparse Matrix visualization
		std::unique_ptr<StandardShader> m_sparseMatrixVis = nullptr;
		std::shared_ptr<ImGuiScreen> m_sparseMatrixWindow = nullptr;
		
		void InitSparseMatrixWindow();
		void ImGuiSparseMatrixWindow();
		bool m_imguiSparseMatrixWindow = false;
		bool m_visSemiImplicitEulerMatrix = false;


		enum class MATRIX_VIEW {
			INV_MASS_MATRIX = 0,
			ENERGY_HESSIAN = 1,
			A = 2
		};
		std::vector<std::string> m_matrixViewStrVec = {
			"Inverse mass matrix",
			"Energy hessian",
			"A = I - beta * dt^2 * H"
		};

		void RenderSparseMatrix(const std::vector<EigenTriplet>& eigenTriplets, std::shared_ptr<ImGuiScreen> imguiScreen, int spMatRows);
		bool m_semiImplicitMatrixIsSymmetric = true;



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

		

		

	public:


		/******************** MATERIAL PARAMETERS EDITOR ********************/
		MaterialParameters m_mpParameters;
		std::vector<MaterialParameters> m_energyModels = std::vector<MaterialParameters>(size_t(ENERGY_MODEL::Count), MaterialParameters());
		std::vector<std::string> m_energyModelsStrVec = {
			"Linear Elasticity",
			"Neohookean Elasticity",
			"Fixed Corotational Elasticity",
			"Snow (Stomakhin et. al 2013)"
		};
		ENERGY_MODEL m_comodel = ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY;
		void ChangeEnergyModel(ENERGY_MODEL);
		

	};
}