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
			mp.m = m;
			mp.vol = vol;
			mp.lam = lam;
			mp.mew = mew;
		}
		
		mat2 F = mat2(1.0);
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
	};

	struct ControlGridNode {
		ControlGridNode() {}
		void Reset_vpm() {
			v = vec2(0.0);
			p = vec2(0.0);
			m = 0.0;
		}
		vec2 x = vec2(0.0); // this will never change, but storing cuz I can afford it

		vec2 v = vec2(0.0);
		vec2 dLdv = vec2(0.0);

		vec2 p = vec2(0.0);
		vec2 dLdp = vec2(0.0);

		real m = 0.0;
		real dLdm = 0.0;
	};

	struct MPMSpaceComputationGraph {
		// use an adjacency list

		// first index is grid node, second is list of particles that node was connected to
		std::vector<std::vector<ControlPoint*>> g2pConnections;
	};

	struct MPMSpaceTimeComputationGraph {

		//std::vector<SparseMatrixd> p2gNetwork;
		std::vector<std::shared_ptr<MPMSpaceComputationGraph>> simStates;
	};

	struct ControlPointCloud {

		ControlPointCloud(std::shared_ptr<const PointCloud> pointCloud);
		ControlPointCloud(std::shared_ptr<const ControlPointCloud> pointCloud);
		~ControlPointCloud() {
			controlPoints.clear();
			glDeleteBuffers(1, &ssbo);
		}

		void SetRegularPointCloud(std::shared_ptr<PointCloud> pointCloud);

		void GenControlPointCloudSSBO();
		void MapToGPU();

		glm::highp_fvec4 color = glm::highp_fvec4(0.f, 1.f, 0.f, 1.f);
		GLuint ssbo = 0;
		std::vector<ControlPoint> controlPoints;
	};

	struct ControlGrid {

		ControlGrid(int _xSize, int _ySize);
		~ControlGrid() {
			nodes.clear(); // vector clears recursively
		}
		int grid_size_x;
		int grid_size_y;
		std::vector<std::vector<ControlGridNode>> nodes;
	};

	void MPMForwardSimulation(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const vec2 f_ext, const size_t timeSteps, const real dt, bool debugOutput);
	void MPMForwardTimeStep(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const vec2 f_ext, const real dt);
	void P2G(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const real dt);
	void G_Update(std::shared_ptr<ControlGrid> grid, const vec2 f_ext, const real dt);
	void G2P(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const real dt);
	void ProjectParticleToGridNode(const ControlPoint& mp, ControlGridNode& node, const real dt);
	void UpdateGridNode(ControlGridNode& node, const vec2 f_ext, const real dt);
	void ProjectGridNodeToParticle(const ControlGridNode& node, ControlPoint& mp, const real dt);
	void UpdateParticle(ControlPoint& mp, const real dt);

	void BackPropGridNodeToParticle(const ControlGridNode& node, ControlPoint& mp, const real dt);

	void ExpandPointCloud(std::shared_ptr<ControlPointCloud> pointCloud, real expansionFactor);

	real PositionLossFunction(std::shared_ptr<const ControlPointCloud> controlPointCloud, std::shared_ptr<const ControlPointCloud> targetPointCloud);
	void OptimizeSetDeformationGradient(std::shared_ptr<ControlPointCloud> controlPointCloud, std::shared_ptr<const ControlPointCloud> targetPointCloud,
										mat2 initialFe, size_t numTimeSteps, size_t max_iters);


	void MapCPUControlPointCloudToGPU(std::shared_ptr<ControlPointCloud> pointCloud);
}
}