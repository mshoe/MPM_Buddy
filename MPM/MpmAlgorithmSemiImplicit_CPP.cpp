#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "MpmAlgorithmEngine.h"
#include "MpmFunctions.h"

void d2Psi_dF2(Eigen::Matrix4d& dPdF, double mew, double lam, const mpm::MaterialPoint& mp) {

	mat2 R, S, U, V;
	real s1, s2;
	PolarDecomp(mp.Fe, R, S);
	SVD(R, S, U, s1, s2, V);

	real u1 = ExtractRotationAngle(U);
	real v1 = ExtractRotationAngle(V);

	// pooped out from MATLAB script: MATLAB/P_SVD.mlx
	// Assuming release mode will optimize this bad boy
	dPdF(0, 0) = mew * cos(u1) * cos(v1) * 2.0 - lam * sin(u1) * sin(v1) + lam * (s2 * s2) * cos(u1) * cos(v1) + lam * s1 * s2 * sin(u1) * sin(v1) * 2.0;
	dPdF(0, 1) = lam * cos(v1) * sin(u1) + mew * cos(u1) * sin(v1) * 2.0 + lam * (s2 * s2) * cos(u1) * sin(v1) - lam * s1 * s2 * cos(v1) * sin(u1) * 2.0;
	dPdF(0, 2) = lam * cos(u1) * sin(v1) + mew * cos(v1) * sin(u1) * 2.0 + lam * (s2 * s2) * cos(v1) * sin(u1) - lam * s1 * s2 * cos(u1) * sin(v1) * 2.0;
	dPdF(0, 3) = -lam * cos(u1) * cos(v1) + mew * sin(u1) * sin(v1) * 2.0 + lam * (s2 * s2) * sin(u1) * sin(v1) + lam * s1 * s2 * cos(u1) * cos(v1) * 2.0;
	dPdF(1, 0) = -(mew * cos(u1) * sin(v1) * -2.0 + mew * cos(v1) * sin(u1) * 2.0 + lam * s1 * cos(v1) * sin(u1) + lam * s2 * cos(v1) * sin(u1) + mew * s1 * cos(u1) * sin(v1) * 2.0 + mew * s2 * cos(u1) * sin(v1) * 2.0 - lam * s1 * (s2 * s2) * cos(v1) * sin(u1) - lam * (s1 * s1) * s2 * cos(v1) * sin(u1)) / (s1 + s2);
	dPdF(1, 1) = -(mew * cos(u1) * cos(v1) * 2.0 + mew * sin(u1) * sin(v1) * 2.0 + lam * s1 * sin(u1) * sin(v1) + lam * s2 * sin(u1) * sin(v1) - mew * s1 * cos(u1) * cos(v1) * 2.0 - mew * s2 * cos(u1) * cos(v1) * 2.0 - lam * s1 * (s2 * s2) * sin(u1) * sin(v1) - lam * (s1 * s1) * s2 * sin(u1) * sin(v1)) / (s1 + s2);
	dPdF(1, 2) = (mew * cos(u1) * cos(v1) * 2.0 + mew * sin(u1) * sin(v1) * 2.0 - mew * s1 * sin(u1) * sin(v1) * 2.0 - mew * s2 * sin(u1) * sin(v1) * 2.0 + lam * s1 * cos(u1) * cos(v1) + lam * s2 * cos(u1) * cos(v1) - lam * s1 * (s2 * s2) * cos(u1) * cos(v1) - lam * (s1 * s1) * s2 * cos(u1) * cos(v1)) / (s1 + s2);
	dPdF(1, 3) = (mew * cos(u1) * sin(v1) * 2.0 - mew * cos(v1) * sin(u1) * 2.0 + lam * s1 * cos(u1) * sin(v1) + lam * s2 * cos(u1) * sin(v1) + mew * s1 * cos(v1) * sin(u1) * 2.0 + mew * s2 * cos(v1) * sin(u1) * 2.0 - lam * s1 * (s2 * s2) * cos(u1) * sin(v1) - lam * (s1 * s1) * s2 * cos(u1) * sin(v1)) / (s1 + s2);
	dPdF(2, 0) = -(mew * cos(u1) * sin(v1) * 2.0 - mew * cos(v1) * sin(u1) * 2.0 + lam * s1 * cos(u1) * sin(v1) + lam * s2 * cos(u1) * sin(v1) + mew * s1 * cos(v1) * sin(u1) * 2.0 + mew * s2 * cos(v1) * sin(u1) * 2.0 - lam * s1 * (s2 * s2) * cos(u1) * sin(v1) - lam * (s1 * s1) * s2 * cos(u1) * sin(v1)) / (s1 + s2);
	dPdF(2, 1) = (mew * cos(u1) * cos(v1) * 2.0 + mew * sin(u1) * sin(v1) * 2.0 - mew * s1 * sin(u1) * sin(v1) * 2.0 - mew * s2 * sin(u1) * sin(v1) * 2.0 + lam * s1 * cos(u1) * cos(v1) + lam * s2 * cos(u1) * cos(v1) - lam * s1 * (s2 * s2) * cos(u1) * cos(v1) - lam * (s1 * s1) * s2 * cos(u1) * cos(v1)) / (s1 + s2);
	dPdF(2, 2) = -(mew * cos(u1) * cos(v1) * 2.0 + mew * sin(u1) * sin(v1) * 2.0 + lam * s1 * sin(u1) * sin(v1) + lam * s2 * sin(u1) * sin(v1) - mew * s1 * cos(u1) * cos(v1) * 2.0 - mew * s2 * cos(u1) * cos(v1) * 2.0 - lam * s1 * (s2 * s2) * sin(u1) * sin(v1) - lam * (s1 * s1) * s2 * sin(u1) * sin(v1)) / (s1 + s2);
	dPdF(2, 3) = (mew * cos(u1) * sin(v1) * -2.0 + mew * cos(v1) * sin(u1) * 2.0 + lam * s1 * cos(v1) * sin(u1) + lam * s2 * cos(v1) * sin(u1) + mew * s1 * cos(u1) * sin(v1) * 2.0 + mew * s2 * cos(u1) * sin(v1) * 2.0 - lam * s1 * (s2 * s2) * cos(v1) * sin(u1) - lam * (s1 * s1) * s2 * cos(v1) * sin(u1)) / (s1 + s2);
	dPdF(3, 0) = -lam * cos(u1) * cos(v1) + mew * sin(u1) * sin(v1) * 2.0 + lam * (s1 * s1) * sin(u1) * sin(v1) + lam * s1 * s2 * cos(u1) * cos(v1) * 2.0;
	dPdF(3, 1) = -lam * cos(u1) * sin(v1) - mew * cos(v1) * sin(u1) * 2.0 - lam * (s1 * s1) * cos(v1) * sin(u1) + lam * s1 * s2 * cos(u1) * sin(v1) * 2.0;
	dPdF(3, 2) = -lam * cos(v1) * sin(u1) - mew * cos(u1) * sin(v1) * 2.0 - lam * (s1 * s1) * cos(u1) * sin(v1) + lam * s1 * s2 * cos(v1) * sin(u1) * 2.0;
	dPdF(3, 3) = mew * cos(u1) * cos(v1) * 2.0 - lam * sin(u1) * sin(v1) + lam * (s1 * s1) * cos(u1) * cos(v1) + lam * s1 * s2 * sin(u1) * sin(v1) * 2.0;
}

void d2e_dxidxj(Eigen::Matrix2d& d2V,
				const Eigen::Vector2d& wpgGrad_i, const Eigen::Vector2d& wpgGrad_j,
				const Eigen::Matrix2d& F,
				double mew, double lam, const mpm::MaterialPoint& mp) {

	Eigen::Matrix4d dPdF; // really a 2x2x2x2 tensor
	d2Psi_dF2(dPdF, mew, lam, mp);

	//F.transpose() * wpgGrad_i;


	// Instead of figuring out how to flatten F and wpg_grad, just use dPdF as a 2x2x2x2 tensor

	for (int a = 0; a < 2; a++) {
		for (int tau = 0; tau < 2; tau++) {
			real value = 0;
			for (int b = 0; b < 2; b++) {
				for (int sig = 0; sig < 2; sig++) {
					real dpdf = dPdF(a + b, tau + sig);

					for (int ome = 0; ome < 2; ome++) {
						real Fwj = F(ome, sig) * wpgGrad_j(ome);
						for (int gam = 0; gam < 2; gam++) {
							real Fwi = F(gam, b) * wpgGrad_i(gam);
							value += dpdf * Fwj * Fwi;
						}
					}
				}
			}
			d2V(a, tau) = value;
		}
	}

	d2V = mp.vol * d2V;
}

typedef Eigen::SparseMatrix<double> SparseMatrixd;
typedef Eigen::Triplet<double> Tripletd;

void EnergyHessian(SparseMatrixd& H, int nodes_x, int nodes_y, const std::map<std::string, std::shared_ptr<mpm::PointCloud>>& pointCloudMap) {
	using namespace mpm;

	// FIRST need to identify grid degrees of freedom (nodes with non-zero mass), otherwise the linear solve Ax = b will not work.

	int numNodes = nodes_x * nodes_y;
	H.resize(2 * numNodes, 2 * numNodes);
	H.setZero();

	std::vector<Tripletd> hessTriplets;
	hessTriplets.reserve(numNodes * 16); // 16 based on cubic b spline basis




	std::vector<Tripletd> massTriplets;
	//for (int i = 0;)

	//std::unordered_map<IntPair, Eigen::Matrix4d> hessianMap;

	//SparseMatrixd
	int x_bound = nodes_x;
	int y_bound = nodes_y;

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : pointCloudMap) {
		for (const MaterialPoint& point : pointCloudPair.second->points) {

			int botLeftNode_row = int(glm::floor(point.x.x)) - 1;
			int botLeftNode_col = int(glm::floor(point.x.y)) - 1;

			vec2 xp = point.x;

			Eigen::Matrix4d dPdF; // really a 2x2x2x2 tensor
			d2Psi_dF2(dPdF, pointCloudPair.second->parameters.mew, pointCloudPair.second->parameters.lam, point);

			// gotta do a QUADRA LOOP for this
			for (int i = 0; i <= 3; i++) {
				for (int ii = 0; ii <= 3; ii++) {

					int currNode_i = botLeftNode_row + i;
					int currNode_ii = botLeftNode_col + ii;

					if (!InBounds(currNode_i, currNode_ii, x_bound, y_bound)) {
						continue;
					}

					vec2 xg_i = vec2(real(currNode_i), real(currNode_ii));
					vec2 dpg_i = xg_i - xp;
					real dx_i = -dpg_i.x; // sign matters for gradient
					real dy_i = -dpg_i.y;
					real wpg_i = CubicBSpline(dx_i) * CubicBSpline(dy_i);
					vec2 wpgGrad_i = vec2(CubicBSplineSlope(dx_i) * CubicBSpline(dy_i), CubicBSpline(dx_i) * CubicBSplineSlope(dy_i));


					size_t index_i = size_t(currNode_i) * size_t(y_bound) + size_t(currNode_ii); // this index is not indexing m_grid.nodes, it is just for the hessian H

					//vec2 vg = m_grid.nodes[index_i].v;

					for (int j = 0; j <= 3; j++) {
						for (int jj = 0; jj <= 3; jj++) {
							int currNode_j = botLeftNode_row + j;
							int currNode_jj = botLeftNode_col + jj;

							if (!InBounds(currNode_j, currNode_jj, x_bound, y_bound)) {
								continue;
							}

							vec2 xg_j = vec2(real(currNode_j), real(currNode_jj));
							vec2 dpg_j = xg_j - xp;
							real dx_j = -dpg_j.x; // sign matters for gradient
							real dy_j = -dpg_j.y;
							real wpg_j = CubicBSpline(dx_j) * CubicBSpline(dy_j);
							vec2 wpgGrad_j = vec2(CubicBSplineSlope(dx_j) * CubicBSpline(dy_j), CubicBSpline(dx_j) * CubicBSplineSlope(dy_j));


							size_t index_j = size_t(currNode_j) * size_t(y_bound) + size_t(currNode_jj);

							Eigen::Matrix2d d2V;
							Eigen::Vector2d wpgGrad_i_eig(wpgGrad_i.x, wpgGrad_i.y);
							Eigen::Vector2d wpgGrad_j_eig(wpgGrad_j.x, wpgGrad_j.y);
							Eigen::Matrix2d F;
							F(0, 0) = point.Fe[0][0];
							F(0, 1) = point.Fe[1][0]; // glm col major
							F(1, 0) = point.Fe[0][1];
							F(1, 1) = point.Fe[1][1];

							d2e_dxidxj(d2V, wpgGrad_i_eig, wpgGrad_j_eig, F, pointCloudPair.second->parameters.mew, pointCloudPair.second->parameters.lam, point);

							hessTriplets.push_back(Tripletd(int(2 * index_i), int(2 * index_j), d2V(0, 0)));
							hessTriplets.push_back(Tripletd(int(2 * index_i + 1), int(2 * index_j), d2V(1, 0)));
							hessTriplets.push_back(Tripletd(int(2 * index_i), int(2 * index_j + 1), d2V(0, 1)));
							hessTriplets.push_back(Tripletd(int(2 * index_i + 1), int(2 * index_j + 1), d2V(1, 1)));
						}
					}
				}
			}
		}
	}

	H.setFromTriplets(hessTriplets.begin(), hessTriplets.end());

}

void GridDegreesOfFreedomVector(std::vector<glm::ivec2>& gridDegreesOfFreedom, int nodes_x, int nodes_y, const mpm::Grid& grid) {
	int numNodes = nodes_x * nodes_y;

	// First identify the grid degrees of freedom
	gridDegreesOfFreedom.clear();
	gridDegreesOfFreedom.reserve(numNodes);
	for (size_t i = 0; i < size_t(nodes_x); i++) {
		for (size_t j = 0; j < size_t(nodes_y); j++) {
			size_t index = i * GRID_SIZE_Y + j;
			//size_t spMatIndex = i * chunks_y * CHUNK_WIDTH + j;

			real nodeMass = grid.nodes[index].m;

			if (nodeMass != 0.0) {
				gridDegreesOfFreedom.push_back(glm::ivec2(i, j));
			}
		}
	}
}

void SelectionMatrix(SparseMatrixd& P, const std::vector<glm::ivec2>& gridDegreesOfFreedom, int nodes_x, int nodes_y, const mpm::Grid& grid) {
	int numNodes = nodes_x * nodes_y;

	// Then put it into a selection matrix
	P.resize(gridDegreesOfFreedom.size() * 2, size_t(numNodes) * 2);
	P.setZero();
	std::vector<Tripletd> selectionTriplets;
	selectionTriplets.reserve(2 * gridDegreesOfFreedom.size());
	for (int i = 0; i < gridDegreesOfFreedom.size(); i++) {
		int nodeIndex = gridDegreesOfFreedom[i].x * nodes_y + gridDegreesOfFreedom[i].y;
		selectionTriplets.push_back(Tripletd(2 * i, 2 * nodeIndex, 1.0));
		selectionTriplets.push_back(Tripletd(2 * i + 1, 2 * nodeIndex + 1, 1.0));
	}

	P.setFromTriplets(selectionTriplets.begin(), selectionTriplets.end());
}

void InverseMassMatrix(SparseMatrixd& M_inv, const std::vector<glm::ivec2>& gridDegreesOfFreedom, int nodes_x, int nodes_y, const mpm::Grid& grid) {
	M_inv.resize(2 * gridDegreesOfFreedom.size(), 2 * gridDegreesOfFreedom.size());
	M_inv.setZero();

	std::vector<Tripletd> massTriplets;
	massTriplets.reserve(gridDegreesOfFreedom.size());
	for (int i = 0; i < gridDegreesOfFreedom.size(); i++) {
		int index = gridDegreesOfFreedom[i].x * GRID_SIZE_Y + gridDegreesOfFreedom[i].y;
		real mass = grid.nodes[index].m;
		massTriplets.push_back(Tripletd(2 * i, 2 * i, 1.0 / mass));
		massTriplets.push_back(Tripletd(2 * i + 1, 2 * i + 1, 1.0 / mass));
	}

	M_inv.setFromTriplets(massTriplets.begin(), massTriplets.end());
}

void GridVelocityToVector(Eigen::VectorXd& v_explicit, const std::vector<glm::ivec2>& gridDegreesOfFreedom, const mpm::Grid& grid) {

	v_explicit.resize(gridDegreesOfFreedom.size() * 2);
	for (size_t i = 0; i < gridDegreesOfFreedom.size(); i++) {
		int index = gridDegreesOfFreedom[i].x * GRID_SIZE_Y + gridDegreesOfFreedom[i].y;
		v_explicit(2 * i) = grid.nodes[index].v.x;
		v_explicit(2 * i + 1) = grid.nodes[index].v.y;
	}
}

void VectorToGridVelocity(mpm::Grid& grid, const Eigen::VectorXd& v_semi_implicit, const std::vector<glm::ivec2>& gridDegreesOfFreedom) {
	for (int i = 0; i < gridDegreesOfFreedom.size(); i++) {
		int index = gridDegreesOfFreedom[i].x * GRID_SIZE_Y + gridDegreesOfFreedom[i].y;
		grid.nodes[index].v.x = v_semi_implicit(2 * size_t(i));
		grid.nodes[index].v.y = v_semi_implicit(2 * size_t(i) + 1);
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepSemiImplicitGridUpdate_CPP(real dt, real beta)
{

	int nodes_x = m_mpmEngine->m_chunks_x * m_cppChunkX;
	int nodes_y = m_mpmEngine->m_chunks_x * m_cppChunkY;

	SparseMatrixd H;
	EnergyHessian(H, nodes_x, nodes_y, m_mpmEngine->m_pointCloudMap);

	std::vector<glm::ivec2> gridDegreesOfFreedom;
	GridDegreesOfFreedomVector(gridDegreesOfFreedom, nodes_x, nodes_y, m_mpmEngine->m_grid);

	SparseMatrixd P;
	SelectionMatrix(P, gridDegreesOfFreedom, nodes_x, nodes_y, m_mpmEngine->m_grid);

	SparseMatrixd M_inv;
	InverseMassMatrix(M_inv, gridDegreesOfFreedom, nodes_x, nodes_y, m_mpmEngine->m_grid);

	Eigen::VectorXd V_symplectic_euler;
	GridVelocityToVector(V_symplectic_euler, gridDegreesOfFreedom, m_mpmEngine->m_grid);

	SparseMatrixd A;
	A.resize(gridDegreesOfFreedom.size() * 2, gridDegreesOfFreedom.size() * 2);
	A.setZero();
	A.setIdentity();

	/*A += beta * dt * dt * M_inv * */

	SparseMatrixd SelectedH = M_inv * P * H * P.transpose();
	A += beta * dt * dt * SelectedH;
	//SparseMatrixd A = Id + beta * dt * dt * M_inv * P * H * P.transpose();


	//Eigen::SimplicialLDLT<SparseMatrixd> chol(A);
	//Eigen::VectorXd V_semi_implict_euler = chol.solve(V_symplectic_euler);

	//std::cout << P << std::endl;

	//std::cout << M_inv << std::endl;

	//std::cout << H << std::endl;

	//std::cout << SelectedH << std::endl;

	//std::cout << A << std::endl;

	Eigen::SparseLU<SparseMatrixd, Eigen::COLAMDOrdering<int>> solver(A);
	solver.compute(A);
	if (solver.info() != Eigen::Success) {
		std::cout << "solver failed." << std::endl;
		m_paused = true;
		return;
	}

	/*Eigen::SimplicialLDLT<SparseMatrixd> solver(A);
	solver.compute(A);
	if (solver.info() != Eigen::Success) {
		std::cout << "solver failed." << std::endl;
		m_paused = true;
		return;
	}*/

	Eigen::VectorXd V_semi_implict_euler = solver.solve(V_symplectic_euler);

	VectorToGridVelocity(m_mpmEngine->m_grid, V_semi_implict_euler, gridDegreesOfFreedom);

}