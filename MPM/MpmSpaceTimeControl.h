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

		ControlPoint() {

		}
		
		ControlPoint(const MaterialPoint& mp) {
			F = mp.Fe;
			dLdF = mat2(0.0);
			P = mp.P;
			dLdP = mat2(0.0);
			C = mp.B;
			dLdC = mat2(0.0);
			x = mp.x;
			dLdx = vec2(0.0);
			v = mp.v;
			dLdv = vec2(0.0);
			m = mp.m;
			vol = mp.vol;
			lam = mp.lam;
			mew = mp.mew;
		}

		void SetRegularMaterialPoint(MaterialPoint& mp) {
			mp.x = x;
			mp.v = v;
			mp.B = C;
			mp.Fe = F + dFc;
			mp.P = P;
			mp.m = m;
			mp.vol = vol;
			mp.lam = lam;
			mp.mew = mew;
		}

		void SetFromPreviousTimeStepControlPoint(const ControlPoint& mp) {
			F = mp.F;
			P = mp.P;
			C = mp.C;
			x = mp.x;
			v = mp.v;
			m = mp.m;
			vol = mp.vol;
			lam = mp.lam;
			mew = mp.mew;
		}

		void ResetGradients() {
			dLdF = mat2(0.0);
			dLdP = mat2(0.0);
			dLdC = mat2(0.0);
			dLdx = vec2(0.0);
			dLdv = vec2(0.0);
		}
		
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
		real vol = 0.0;
		real lam = 0.0;
		real mew = 0.0;

		void ImGuiDisplay() {
			glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
			glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);

			ImGui::DisplayNamedGlmVecMixColor("x", x, min_color, max_color);
			ImGui::DisplayNamedGlmVecMixColor("dLdx", dLdx, min_color, max_color);
			ImGui::DisplayNamedGlmVecMixColor("v", v, min_color, max_color);
			ImGui::DisplayNamedGlmVecMixColor("dLdc", dLdv, min_color, max_color);
			ImGui::DisplayNamedGlmRealColor("m", m, max_color);
			ImGui::DisplayNamedGlmRealColor("vol", vol, max_color);
			ImGui::DisplayNamedGlmRealColor("lam", lam, max_color);
			ImGui::DisplayNamedGlmRealColor("mew", mew, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("F", F, min_color, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("dFc", dFc, min_color, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("dLdF", dLdF, min_color, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("P", P, min_color, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("dLdP", dLdP, min_color, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("C", C, min_color, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("dLdC", dLdC, min_color, max_color);
		}
	};

	struct ControlGridNode {
		ControlGridNode() {}
		void Reset_vpm() {
			v = vec2(0.0);
			p = vec2(0.0);
			m = 0.0;
		}
		void ResetGradients() {
			dLdv = vec2(0.0);
			dLdp = vec2(0.0);
			dLdm = 0.0;
		}

		vec2 x = vec2(0.0); // this will never change, but storing cuz I can afford it

		vec2 v = vec2(0.0);
		vec2 dLdv = vec2(0.0);

		vec2 p = vec2(0.0);
		vec2 dLdp = vec2(0.0);

		real m = 0.0;
		real dLdm = 0.0;
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
		void SetF(mat2 F);
		void ResetGradients() {
			for (size_t p = 0; p < controlPoints.size(); p++) {
				controlPoints[p].ResetGradients();
			}
		}
		void ResetdFc() {
			for (size_t p = 0; p < controlPoints.size(); p++) {
				controlPoints[p].dFc = mat2(0.0);
			}
		}
		void DescendFGradients(real alpha) {
			for (ControlPoint& mp : controlPoints) {
				// view this dLdF matrix as a vector
				// descend in the direction of the vector
				mat2 dLdF_dir = NormalizedMatrix(mp.dLdF);
				mp.dFc -= alpha * dLdF_dir;
			}
		}

		glm::highp_fvec4 color = glm::highp_fvec4(0.f, 1.f, 0.f, 1.f);
		std::vector<ControlPoint> controlPoints;
	};

	struct ControlGrid {

		ControlGrid(int _xSize, int _ySize);
		~ControlGrid() {
			nodes.clear(); // vector clears recursively
		}

		void ResetGradients() {
			for (size_t i = 0; i < size_t(grid_size_x); i++) {
				for (size_t j = 0; j < size_t(grid_size_y); j++) {
					nodes[i][j].ResetGradients();
				}
			}
		}

		int grid_size_x;
		int grid_size_y;
		std::vector<std::vector<ControlGridNode>> nodes;
	};

	typedef std::vector<std::vector<std::vector<size_t>>> G2PNetwork;

	struct MPMSpaceComputationGraph {
		MPMSpaceComputationGraph(std::shared_ptr<ControlPointCloud> _pointCloud, std::shared_ptr<ControlGrid> _grid) {
			pointCloud = _pointCloud;
			grid = _grid;

			g2pnetwork.resize(
				grid->grid_size_x,
				std::vector<std::vector<size_t>>(
					grid->grid_size_y,
					std::vector<size_t>{}
			));
		}

		std::shared_ptr<ControlPointCloud> pointCloud = nullptr;
		std::shared_ptr<ControlGrid> grid = nullptr;

		// use an adjacency list

		// [i][j] is grid node, [k] is list of particles that node was connected to
		G2PNetwork g2pnetwork;
	};

	struct MPMSpaceTimeComputationGraph {

		~MPMSpaceTimeComputationGraph() {
			glDeleteBuffers(1, &controlSsbo);
			glDeleteBuffers(1, &targetSsbo);
		}

		void InitSTCG();
		void SetGridSize(int _grid_size_x, int _grid_size_y) {
			grid_size_x = _grid_size_x;
			grid_size_y = _grid_size_y;
		}

		std::shared_ptr<ControlPointCloud> originalPointCloud = nullptr;
		std::shared_ptr<ControlPointCloud> controlPointCloud = nullptr;
		std::shared_ptr<ControlPointCloud> targetPointCloud = nullptr;
		std::shared_ptr<ControlPointCloud> outputPointCloud = nullptr;

		mat2 controlF = mat2(1.0);

		//std::vector<SparseMatrixd> p2gNetwork;
		int timeSteps = 120;
		int iters = 10;
		int grid_size_x = 32;
		int grid_size_y = 32;
		std::vector<std::shared_ptr<MPMSpaceComputationGraph>> simStates;

		//void OptimizeControlF();

		void InitControlPointCloud(std::shared_ptr<PointCloud> pointCloud);
		void InitTargetPointCloud(std::shared_ptr<ControlPointCloud> pointCloud);

		
		//void MapToGPU(std::shared_ptr<ControlPointCloud> pointCloud, GLuint ssbo);

		//GLuint originalSsbo = 0;
		GLuint controlSsbo = 0;
		GLuint targetSsbo = 0;
	};

	// FORWARD SIMULATION
	void MPMForwardSimulation(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg, const vec2 f_ext, const real dt, bool debugOutput);
	void MPMForwardTimeStep(std::shared_ptr<MPMSpaceComputationGraph> scg_n, 
							std::shared_ptr<MPMSpaceComputationGraph> scg_nplus1,
							const vec2 f_ext, const real dt);
	void P2G(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const real dt);
	void G_Update(std::shared_ptr<ControlGrid> grid, const vec2 f_ext, const real dt);
	void G2P(std::shared_ptr<const ControlPointCloud> pointCloud_n,
			 std::shared_ptr<ControlPointCloud> pointCloud_nplus1,
			 std::shared_ptr<ControlGrid> grid, const real dt);
	void ProjectParticleToGridNode(const ControlPoint& mp, ControlGridNode& node, const real dt);
	void UpdateGridNode(ControlGridNode& node, const vec2 f_ext, const real dt);
	void ProjectGridNodeToParticle(const ControlGridNode& node, ControlPoint& mp, const real dt);
	void UpdateParticle(ControlPoint& mp, const ControlPoint& mp_prev, const real dt);


	// BACKPROPOGATION
	void MPMBackPropogation(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg, const real dt, bool debugOutput);
	void BackPropParticleInit(ControlPoint& mp, const ControlPoint& mp_target, const real dt);

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
	
	real PositionLossFunction(std::shared_ptr<const ControlPointCloud> controlPointCloud, std::shared_ptr<const ControlPointCloud> targetPointCloud);
	void OptimizeSetDeformationGradient(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg,
										const vec2 f_ext, const real dt,
										mat2 initialFe, int optFrameOffset,
										int numTimeSteps, int max_iters, int maxLineSearchIters,
										real initialAlpha, bool optimizeOnlyInitialF, bool debugOutput);

	void GenControlPointCloudSSBO(std::shared_ptr<ControlPointCloud> pointCloud, GLuint& ssbo);
	void MapCPUControlPointCloudToGPU(std::shared_ptr<ControlPointCloud> pointCloud, GLuint ssbo);
}
}