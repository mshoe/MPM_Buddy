#pragma once

//#include <Eigen/Dense>
//#include <Eigen/Sparse>

#include "Structures.h"
#include "Constants.h"
#include "MpmFunctions.h"
#include "EnergyFunctions.h"
#include "PointCloud.h"
#include <vector>
#include <iostream>

namespace mpm {
namespace control {

	//typedef Eigen::SparseMatrix<double> SparseMatrixd;

	struct ControlPoint {

		ControlPoint() {}
		
		ControlPoint(const MaterialPoint& mp);

		void SetRegularMaterialPoint(MaterialPoint& mp);

		void SetFromPreviousTimeStepControlPoint(const ControlPoint& mp);

		void ResetGradients();
		
		mat2 F = mat2(1.0);
		mat2 dFc = mat2(0.0);
		mat2 dLdF = mat2(0.0);

		mat2 P = mat2(0.0);
		mat2 dLdP = mat2(0.0);

		mat2 C = mat2(0.0);
		mat2 dLdC = mat2(0.0);

		vec2 x = vec2(0.0);
		vec2 dLdx = vec2(0.0);

		vec2 v = vec2(0.0);
		vec2 dLdv = vec2(0.0);

		real m = 0.0;
		real dmc = 0.0;
		real dLdm = 0.0;

		real vol = 0.0;
		real dvolc = 0.0;
		real dLdvol = 0.0;

		real lam = 0.0;
		real dlamc = 0.0;
		real dLdlam = 0.0;

		real mew = 0.0;
		real dmewc = 0.0;
		real dLdmew = 0.0;

		void ImGuiDisplay();
	};

	struct ControlGridNode {
		ControlGridNode() {}
		void Reset_vpm();

		void ResetGradients();

		vec2 x = vec2(0.0); // this will never change, but storing cuz I can afford it

		vec2 v = vec2(0.0);
		vec2 dLdv = vec2(0.0);

		vec2 p = vec2(0.0);
		vec2 dLdp = vec2(0.0);

		real m = 0.0;
		real dLdm = 0.0;

		void ImGuiDisplay();
	};

	

	struct ControlPointCloud {

		ControlPointCloud(std::shared_ptr<const PointCloud> pointCloud);
		ControlPointCloud(std::shared_ptr<const ControlPointCloud> pointCloud);
		~ControlPointCloud() {
			controlPoints.clear();
		}

		void SetFromControlPointCloud(std::shared_ptr<const ControlPointCloud> pointCloud);
		void SetFromPreviousTimeStepControlPointCloud(std::shared_ptr<const ControlPointCloud> pointCloud);
		void SetRegularPointCloud(std::shared_ptr<PointCloud> pointCloud);
		//std::shared_ptr<PointCloud> GenRegularPointCloud();
		void SetF(mat2 F);
		void ResetGradients();
		void ResetdFc();

		bool CheckFConvergence(real tol, bool debugOutput);

		void DescendFGradients(real alpha);
		
		void DescendMaterialGradients(real alpha);

		void ComputeTotalMass();
		void SetPointCloudMassEqualToGiven(std::shared_ptr<const ControlPointCloud> pointCloud);

		void SetFsToIdentity();

		bool Check_dLdF_Nan();
		//ControlPoint& GetLargestFGradientPoint() {
		//	real max_dLdF_mag = -1.0;
		//	ControlPoint& max_mp = controlPoints[0];
		//	for (ControlPoint& mp : controlPoints) {
		//		// view this dLdF matrix as a vector
		//		// descend in the direction of the vector
		//		real dLdF_mag = MatrixNormSqrd(mp.dLdF);
		//		if (dLdF_mag > max_dLdF_mag) {
		//			max_dLdF_mag = dLdF_mag;
		//			max_mp = mp;
		//		}
		//	}
		//	return max_mp;
		//}

		real totalMass = 0.0;
		glm::highp_fvec4 color = glm::highp_fvec4(0.f, 1.f, 0.f, 1.f);
		std::vector<ControlPoint> controlPoints;
	};

	struct ControlGrid {
		ControlGrid() {}
		ControlGrid(int _xSize, int _ySize);
		virtual ~ControlGrid();

		void InitGrid(int _xSize, int _ySize);

		ControlGridNode& Node(size_t i, size_t j);

		const ControlGridNode& ConstNode(size_t i, size_t j) const;

		void ResetGradients();
		real GetTotalMass();

		int grid_size_x;
		int grid_size_y;
		std::vector<ControlGridNode> nodes;
	};

	struct TargetGrid : public ControlGrid {

		TargetGrid(int _xSize, int _ySize);
		~TargetGrid();

		void InitializePenaltyWeights(real penalty);

		std::vector<std::vector<real>> penaltyWeights;
	};

	struct MPMSpaceComputationGraph {
		MPMSpaceComputationGraph(std::shared_ptr<ControlPointCloud> _pointCloud, std::shared_ptr<ControlGrid> _grid) {
			pointCloud = _pointCloud;
			grid = _grid;
		}

		std::shared_ptr<ControlPointCloud> pointCloud = nullptr;
		std::shared_ptr<ControlGrid> grid = nullptr;
	};

	struct MPMSpaceTimeComputationGraph {

		MPMSpaceTimeComputationGraph();

		~MPMSpaceTimeComputationGraph();

		void InitSTCG();
		void SetGridSize(int _grid_size_x, int _grid_size_y);

		std::shared_ptr<ControlPointCloud> originalPointCloud = nullptr;
		std::shared_ptr<ControlPointCloud> controlPointCloud = nullptr;
		std::shared_ptr<ControlPointCloud> targetPointCloud = nullptr;
		std::shared_ptr<ControlPointCloud> outputPointCloud = nullptr;

		std::shared_ptr<TargetGrid> targetGrid = nullptr;

		mat2 controlF = mat2(1.0);

		bool bounded_dFc = false;
		real max_dFc_norm = 1.0;
		
		int timeSteps = 120;
		int iters = 10;
		int grid_size_x = 32;
		int grid_size_y = 32;
		std::vector<std::shared_ptr<MPMSpaceComputationGraph>> simStates;

		void InitControlPointCloud(std::shared_ptr<PointCloud> pointCloud);
		void InitControlPointCloud(std::shared_ptr<ControlPointCloud> pointCloud);
		void InitTargetPointCloud(std::shared_ptr<PointCloud> pointCloud);
		void InitTargetPointCloud(std::shared_ptr<ControlPointCloud> pointCloud);

		void SetTargetPointCloudMassToControl();

		std::vector<float> lossValues;

		GLuint controlSsbo = 0;
		GLuint targetSsbo = 0;

		GLuint targetGridSsbo = 0;
		GLuint gridSsbo = 0;
	};

	// FORWARD SIMULATION
	void MPMForwardSimulation(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg, 
							  const vec2 f_ext, const real dt, 
							  int controlTimeStep,
							  bool debugOutput);
	void MPMForwardTimeStep(std::shared_ptr<MPMSpaceComputationGraph> scg_n, 
							std::shared_ptr<MPMSpaceComputationGraph> scg_nplus1,
							const vec2 f_ext, const real dt);
	void P2G(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const real dt);
	void G_Update(std::shared_ptr<ControlGrid> grid, const vec2 f_ext, const real dt);
	void G2P(std::shared_ptr<const ControlPointCloud> pointCloud_n,
			 std::shared_ptr<ControlPointCloud> pointCloud_nplus1,
			 std::shared_ptr<const ControlGrid> grid, const real dt);
	void ProjectParticleToGridNode(const ControlPoint& mp, ControlGridNode& node, const real dt);
	void UpdateGridNode(ControlGridNode& node, const vec2 f_ext, const real dt);
	void ProjectGridNodeToParticle(const ControlGridNode& node, ControlPoint& mp, const real dt);
	void UpdateParticle(ControlPoint& mp, const ControlPoint& mp_prev, const real dt);

	const std::vector<std::string> lossFunctionStrVec = {
		"Particle positions",
		"Grid node masses"
	};
	enum class LOSS_FUNCTION {
		PARTICLE_POSITIONS = 0,
		GRID_NODE_MASSES = 1
	};
	enum class LOSS_PENALTY {
		MP_VELOCITIES = 0,
		MP_DEFGRADS = 1
	};

	// BACKPROPOGATION
	void MPMBackPropogation(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg, 
							const real dt, 
							LOSS_FUNCTION lossFunction,
							int controlTimeStep,
							bool debugOutput);

	void BackPropPointCloudPositionLossFunctionInit(std::shared_ptr<ControlPointCloud> controlPointCloud,
													std::shared_ptr<const ControlPointCloud> targetPointCloud,
													const real dt);
	void BackPropParticlePositionLossFunctionInit(ControlPoint& mp, const ControlPoint& mp_target, const real dt);

	void BackPropGridMassLossFunctionInit(std::shared_ptr<ControlPointCloud> controlPointCloud,
										  std::shared_ptr<ControlGrid> controlGrid,
										  std::shared_ptr<const TargetGrid> targetGrid,
										  const real dt);


	void MPMBackPropogationTimeStep(std::shared_ptr<MPMSpaceComputationGraph> scg_nplus1,
									std::shared_ptr<MPMSpaceComputationGraph> scg_n,
									const real dt);
	
	void BackG2P(std::shared_ptr<const ControlPointCloud> pointCloud_nplus1, 
				 std::shared_ptr<const ControlPointCloud> pointCloud_n,
				 std::shared_ptr<ControlGrid> grid, const real dt);
	void BackG_Update(std::shared_ptr<ControlGrid> grid);
	void BackP2G(std::shared_ptr<const ControlPointCloud> pointCloud_nplus1,
				 std::shared_ptr<ControlPointCloud> pointCloud_n,
				 std::shared_ptr<const ControlGrid> grid, const real dt);
	void BackPropParticleToGridNode(const ControlPoint& mp, const ControlPoint& mp_prev, ControlGridNode& node, const real dt);
	void BackPropGridNode(ControlGridNode& node);
	void BackPropGridNodeToParticle(const ControlGridNode& node, ControlPoint& mp_prev, const real dt);
	void BackPropParticleToParticle(const ControlPoint& mp, ControlPoint& mp_prev, const real dt);

	void ExpandPointCloud(std::shared_ptr<ControlPointCloud> pointCloud, real expansionFactor);
	void TranslatePointCloud(std::shared_ptr<ControlPointCloud> pointCloud, vec2 translation);
	void TransformPointCloud(std::shared_ptr<ControlPointCloud> pointCloud, mat2 transformMat);
	
	real ComputeLoss(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg,
					 LOSS_FUNCTION lossFunction,
					 const real dt);

	real ParticlePositionLossFunction(std::shared_ptr<const ControlPointCloud> controlPointCloud, 
									  std::shared_ptr<const ControlPointCloud> targetPointCloud);

	real GridMassLossFunction(std::shared_ptr<ControlPointCloud> controlPointCloud,
							  std::shared_ptr<ControlGrid> controlGrid,
							  std::shared_ptr<const TargetGrid> targetGrid,
							  const real dt);

	real GridMassLossFunction_WithPenalty(std::shared_ptr<ControlPointCloud> controlPointCloud,
										  std::shared_ptr<ControlGrid> controlGrid,
										  std::shared_ptr<const TargetGrid> targetGrid,
										  const std::vector<LOSS_PENALTY> &penalties,
										  const real dt);

	real MpDefGradPenalty(std::shared_ptr<ControlPointCloud> controlPointCloud);
	real MpVelocityPenalty(std::shared_ptr<ControlPointCloud> controlPointCloud);


	/*void OptimizeSetDeformationGradient(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg,
										const vec2 f_ext, const real dt,
										mat2 initialFe, int optFrameOffset,
										int numTimeSteps, int max_iters, int maxLineSearchIters,
										LOSS_FUNCTION lossFunction, bool forceDescent,
										real penalty,
										real initialAlpha, bool optimizeOnlyInitialF, bool debugOutput);*/

	void OptimizeSetDeformationGradient_InTemporalOrder(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg,
														const vec2 f_ext, const real dt,
														bool setInitialFe, mat2 initialFe, 
														int optFrameOffset,
														int numTimeSteps, int max_iters, int maxLineSearchIters,
														int totalTemporalIterations,
														LOSS_FUNCTION lossFunction, bool forceDescent,
														bool reverseTime, real penalty,
														real initialAlpha, real initialMaterialAlpha,
														real tol, real suffTemporalIterLossDecreaseFactor,
														bool optimizeOnlyInitialF, bool debugOutput);


	void GenControlPointCloudSSBO(std::shared_ptr<ControlPointCloud> pointCloud, GLuint& ssbo);
	void GenControlGridSSBO(std::shared_ptr<ControlGrid> grid, GLuint& ssbo);
	void MapCPUControlPointCloudToGPU(std::shared_ptr<ControlPointCloud> pointCloud, GLuint ssbo);
	void MapCPUControlGridToGPU(std::shared_ptr<ControlGrid> grid, GLuint ssbo);
}
}