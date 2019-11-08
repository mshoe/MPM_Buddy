#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "MpmEngine.h"



bool InBounds(int node_i, int node_j, int x_bound, int y_bound) {
	return (node_i >= 0 && node_i < x_bound && node_j >= 0 && node_j < y_bound);
}

real CubicBSpline(real x) {
	using glm::step;
	x = abs(x);
	return (x < 1.0) ? step(0.0, x) * (0.5 * x * x * x - x * x + 2.0 / 3.0) :
		step(x, 2.0) * (2.0 - x) * (2.0 - x) * (2.0 - x) / 6.0;
}

real CubicBSplineSlope(real x) {
	using glm::step;
	real absx = abs(x);
	return (absx < 1.0) ? step(0.0, absx) * (1.5 * x * absx - 2 * x) :
		step(absx, 2.0) * (-x * absx / 2 + 2 * x - 2 * x / absx);
}

void PolarDecomp(const mat2 &F, mat2 &R, mat2 &S) {
	// calculate the polar decomposition F = RS.

	real x = F[0][0] + F[1][1];
	real y = F[0][1] - F[1][0]; // glsl is column major. This is really F_21 - F_12 in row major notation

	real d = sqrt(x * x + y * y);

	real c = x / d;
	real s = -y / d;
	R = (d == 0.0) ? mat2(1.0, 0.0, 0.0, 1.0) : mat2(c, -s, s, c);
	S = transpose(R) * F;
}

void SVD(const mat2 &R, const mat2 &S, mat2 &U, real &sig1, real &sig2, mat2 &V) {

	// check if S is diagonal (S is symmetric for sure)
	double c_hat, s_hat;

	if (S[1][0] == 0) {
		c_hat = 1.0;
		s_hat = 0.0;
		sig1 = S[0][0];
		sig2 = S[1][1];
	}
	else {
		double tau = 0.5 * (S[0][0] - S[1][1]);
		double w = sqrt(tau * tau + S[1][0] * S[1][0]);
		double t = (tau > 0) ? S[1][0] / (tau + w) : S[1][0] / (tau - w);
		c_hat = 1.0 / sqrt(t * t + 1.0);
		s_hat = -t * c_hat;
		sig1 = c_hat * c_hat * S[0][0] - 2.0 * c_hat * s_hat * S[1][0] + s_hat * s_hat * S[1][1];
		sig2 = s_hat * s_hat * S[0][0] + 2.0 * c_hat * s_hat * S[1][0] + c_hat * c_hat * S[1][1];
	}

	double c, s;
	if (sig1 < sig2) {
		// swap the singular values so sig1 > sig2
		double temp = sig1;
		sig1 = sig2;
		sig2 = temp;
		c = -s_hat;
		s = c_hat;
	}
	else {
		c = c_hat;
		s = s_hat;
	}
	V = mat2(c, -s, s, c);
	U = R * V;
}

double ExtractRotationAngle(mat2 R) {
	// assume R is a rotation matrix
	/*
	|cos(a), -sin(a)|
	|sin(a), cos(a) |
	*/

	return atan2(R[0][1], R[0][0]); // glm is column major
}

void mpm::MpmEngine::MpmReset_CPP()
{
	m_pointCloudMap.clear();
	m_timeStep = 0;
	m_time = 0.0;

	for (size_t i = 0; i < size_t(m_chunks_x) * size_t(CHUNK_WIDTH); i++) {
		for (size_t j = 0; j < size_t(m_chunks_y) * size_t(CHUNK_WIDTH); j++) {
			size_t index = i * GRID_SIZE_Y + j;
			m_grid.nodes[index].m = 0.0;
			m_grid.nodes[index].v = vec2(0.0);
			m_grid.nodes[index].momentum = vec2(0.0);
			m_grid.nodes[index].force = vec2(0.0);
		}
	}
}

void mpm::MpmEngine::MpmTimeStep_CPP(real dt)
{
#ifdef MPM_CPP_DEBUG
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();
#endif
	MpmTimeStepP2G_CPP(dt);

	MpmTimeStepExplicitGridUpdate_CPP(dt);

	if (m_semiImplicitCPP) {
		MpmTimeStepSemiImplicitGridUpdate_CPP(dt, m_beta);
	}

	MpmTimeStepG2P_CPP(dt);

	m_timeStep++;
	m_time += m_dt;
#ifdef MPM_CPP_DEBUG
	t2 = high_resolution_clock::now();
	std::cout << "Finished calculating timestep in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
#endif
}

void mpm::MpmEngine::MpmTimeStepP2G_CPP(real dt)
{
	// reset the grid
	for (size_t i = 0; i < size_t(m_chunks_x) * size_t(CHUNK_WIDTH); i++) {
		for (size_t j = 0; j < size_t(m_chunks_y) * size_t(CHUNK_WIDTH); j++) {
			size_t index = i * GRID_SIZE_Y + j;
			m_grid.nodes[index].m = 0.0;
			m_grid.nodes[index].v = vec2(0.0);
			m_grid.nodes[index].momentum = vec2(0.0);
			m_grid.nodes[index].force = vec2(0.0);
		}
	}

	int x_bound = m_chunks_x * CHUNK_WIDTH;
	int y_bound = m_chunks_y * CHUNK_WIDTH;

	static const real Dp_inv = 3.0; // actually 3 * identity matrix

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {

		for (const MaterialPoint &point : pointCloudPair.second->points) {


			int botLeftNode_i = int(glm::floor(point.x.x)) - 1;
			int botLeftNode_j = int(glm::floor(point.x.y)) - 1;

			vec2 xp = point.x;

			for (int i = 0; i <= 3; i++) {
				for (int j = 0; j <= 3; j++) {

					int currNode_i = botLeftNode_i + i;
					int currNode_j = botLeftNode_j + j;

					if (!InBounds(currNode_i, currNode_j, x_bound, y_bound)) {
						continue;
					}

					vec2 xg = vec2(real(currNode_i), real(currNode_j));
					vec2 dpg = xg - xp;
					real dx = -dpg.x; // sign matters for gradient
					real dy = -dpg.y;
					real wpg = CubicBSpline(dx) * CubicBSpline(dy);
					vec2 wpgGrad = vec2(CubicBSplineSlope(dx) * CubicBSpline(dy), 
										CubicBSpline(dx) * CubicBSplineSlope(dy));
					
					size_t index = size_t(currNode_i) * size_t(GRID_SIZE_Y) + size_t(currNode_j);

					// P2G mass transfer
					m_grid.nodes[index].m += wpg * point.m;

					// P2G APIC momentum transfer
					m_grid.nodes[index].momentum += wpg * point.m * (point.v + point.B * Dp_inv * dpg);

					// P2G force transfer
					m_grid.nodes[index].force += -point.vol * point.P * glm::transpose(point.Fe) * wpgGrad;
				}
			}
		}
	}
}

void mpm::MpmEngine::MpmTimeStepExplicitGridUpdate_CPP(real dt)
{
	for (size_t i = 0; i < size_t(m_chunks_x) * size_t(CHUNK_WIDTH); i++) {
		for (size_t j = 0; j < size_t(m_chunks_y) * size_t(CHUNK_WIDTH); j++) {
			size_t index = i * GRID_SIZE_Y + j;

			real nodeMass = m_grid.nodes[index].m;

			if (nodeMass == 0.0)
				continue;

			vec2 xg = vec2(real(i), real(j));
			vec2 nodeMomentum = m_grid.nodes[index].momentum;
			vec2 nodeForce = m_grid.nodes[index].force;
			vec2 mouseForce = m_mousePower * real(m_mouseMpmRenderScreenGridSpaceFull.w) * glm::normalize(vec2(m_mouseMpmRenderScreenGridSpace.x - xg.x, m_mouseMpmRenderScreenGridSpace.y - xg.y));
			// ignoring (experimental) nodal acceleration
			
			vec2 gridV = nodeMomentum / nodeMass;
			vec2 gridAcc = nodeForce / nodeMass;
			vec2 gridUpdateV = gridV * (1.0 - dt * m_drag) + dt * (gridAcc + m_globalForce + mouseForce);

			m_grid.nodes[index].v = gridUpdateV;

		}
	}
}

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
	const Eigen::Vector2d &wpgGrad_i, const Eigen::Vector2d &wpgGrad_j,
	const Eigen::Matrix2d &F,
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

void EnergyHessian(SparseMatrixd& H, int chunks_x, int chunks_y, const std::map<std::string, std::shared_ptr<mpm::PointCloud>> &pointCloudMap) {
	using namespace mpm;

	// FIRST need to identify grid degrees of freedom (nodes with non-zero mass), otherwise the linear solve Ax = b will not work.

	int numNodes = chunks_x * CHUNK_WIDTH * chunks_y * CHUNK_WIDTH;
	H.resize(2 * numNodes, 2 * numNodes);
	H.setZero();

	std::vector<Tripletd> hessTriplets;
	hessTriplets.reserve(numNodes * 16); // 16 based on cubic b spline basis




	std::vector<Tripletd> massTriplets;
	//for (int i = 0;)

	//std::unordered_map<IntPair, Eigen::Matrix4d> hessianMap;

	//SparseMatrixd
	int x_bound = chunks_x * CHUNK_WIDTH;
	int y_bound = chunks_y * CHUNK_WIDTH;

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

void GridDegreesOfFreedomVector(std::vector<glm::ivec2>& gridDegreesOfFreedom, int chunks_x, int chunks_y, const mpm::Grid& grid) {
	int numNodes = chunks_x * CHUNK_WIDTH * chunks_y * CHUNK_WIDTH;

	// First identify the grid degrees of freedom
	gridDegreesOfFreedom.clear();
	gridDegreesOfFreedom.reserve(numNodes);
	for (size_t i = 0; i < size_t(chunks_x) * size_t(CHUNK_WIDTH); i++) {
		for (size_t j = 0; j < size_t(chunks_y) * size_t(CHUNK_WIDTH); j++) {
			size_t index = i * GRID_SIZE_Y + j;
			//size_t spMatIndex = i * chunks_y * CHUNK_WIDTH + j;

			real nodeMass = grid.nodes[index].m;

			if (nodeMass != 0.0) {
				gridDegreesOfFreedom.push_back(glm::ivec2(i, j));
			}
		}
	}
}

void SelectionMatrix(SparseMatrixd& P, const std::vector<glm::ivec2>& gridDegreesOfFreedom, int chunks_x, int chunks_y, const mpm::Grid &grid) {
	int numNodes = chunks_x * CHUNK_WIDTH * chunks_y * CHUNK_WIDTH;

	// Then put it into a selection matrix
	P.resize(gridDegreesOfFreedom.size() * 2, size_t(numNodes) * 2);
	P.setZero();
	std::vector<Tripletd> selectionTriplets;
	selectionTriplets.reserve(2 * gridDegreesOfFreedom.size());
	for (int i = 0; i < gridDegreesOfFreedom.size(); i++) {
		int nodeIndex = gridDegreesOfFreedom[i].x * chunks_y * CHUNK_WIDTH + gridDegreesOfFreedom[i].y;
		selectionTriplets.push_back(Tripletd(2*i, 2*nodeIndex, 1.0));
		selectionTriplets.push_back(Tripletd(2*i + 1, 2*nodeIndex + 1, 1.0));
	}

	P.setFromTriplets(selectionTriplets.begin(), selectionTriplets.end());
}

void InverseMassMatrix(SparseMatrixd &M_inv, const std::vector<glm::ivec2>& gridDegreesOfFreedom, int chunks_x, int chunks_y, const mpm::Grid& grid) {
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

void GridVelocityToVector(Eigen::VectorXd &v_explicit, const std::vector<glm::ivec2>& gridDegreesOfFreedom, const mpm::Grid& grid) {

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

void mpm::MpmEngine::MpmTimeStepSemiImplicitGridUpdate_CPP(real dt, real beta)
{
	
	SparseMatrixd H;
	EnergyHessian(H, m_chunks_x, m_chunks_y, m_pointCloudMap);

	std::vector<glm::ivec2> gridDegreesOfFreedom;
	GridDegreesOfFreedomVector(gridDegreesOfFreedom, m_chunks_x, m_chunks_y, m_grid);

	SparseMatrixd P;
	SelectionMatrix(P, gridDegreesOfFreedom, m_chunks_x, m_chunks_y, m_grid);

	SparseMatrixd M_inv;
	InverseMassMatrix(M_inv, gridDegreesOfFreedom, m_chunks_x, m_chunks_y, m_grid);

	Eigen::VectorXd V_symplectic_euler;
	GridVelocityToVector(V_symplectic_euler, gridDegreesOfFreedom, m_grid);

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

	VectorToGridVelocity(m_grid, V_semi_implict_euler, gridDegreesOfFreedom);
	
}

void mpm::MpmEngine::MpmTimeStepG2P_CPP(real dt)
{
	int x_bound = m_chunks_x * CHUNK_WIDTH;
	int y_bound = m_chunks_y * CHUNK_WIDTH;

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {

		for (MaterialPoint &point : pointCloudPair.second->points) {


			int botLeftNode_i = int(glm::floor(point.x.x)) - 1;
			int botLeftNode_j = int(glm::floor(point.x.y)) - 1;

			vec2 xp = point.x;

			vec2 vp = vec2(0.0);
			mat2 bp = mat2(0.0);
			mat2 dfp = mat2(1.0);

			for (int i = 0; i <= 3; i++) {
				for (int j = 0; j <= 3; j++) {

					int currNode_i = botLeftNode_i + i;
					int currNode_j = botLeftNode_j + j;

					if (!InBounds(currNode_i, currNode_j, x_bound, y_bound)) {
						continue;
					}

					vec2 xg = vec2(real(currNode_i), real(currNode_j));
					vec2 dpg = xg - xp;
					real dx = -dpg.x; // sign matters for gradient
					real dy = -dpg.y;
					real wpg = CubicBSpline(dx) * CubicBSpline(dy);
					vec2 wpgGrad = vec2(CubicBSplineSlope(dx) * CubicBSpline(dy), CubicBSpline(dx) * CubicBSplineSlope(dy));


					size_t index = size_t(currNode_i) * size_t(GRID_SIZE_Y) + size_t(currNode_j);

					vec2 vg = m_grid.nodes[index].v;

					// for deformation gradient update
					dfp += dt * outerProduct(vg, wpgGrad);

					// for material point velocity update
					vp += wpg * vg;

					// APIC
					bp += wpg * outerProduct(vg, dpg);
				}
			}

			point.v = vp;
			point.B = bp; // for APIC
			point.x += dt * vp; // update position


			real mew = point.mew;
			real lam = point.lam;

			// cant put in switch statement because it will be redefinition error
			mat2 Fe;
			mat2 Fit;
			mat2 Fp;
			real J;
			real logJ;
			mat2 R;
			mat2 S;

			// for snow
			mat2 F_total;
			mat2 U;
			real sig1;
			real sig2;
			mat2 V;
			real crit_c;
			real crit_s;
			mat2 sigMat;
			mat2 Feit;
			real Je;
			real Jp;
			real hardening;
			real pcof;
			
			// ignoring extra visualization and debugging stuff for now
			switch (m_comodel) {
				case ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY:
					point.Fe = dfp * point.Fe;
					Fe = point.Fe;
					Fit = glm::transpose(glm::inverse(Fe));
					J = glm::determinant(Fe);
					logJ = real(glm::log(float(J))); // doing it this way cuz glsl doesn't support log on doubles
					point.P = mew * (Fe - Fit) + lam * logJ * Fit;

					
					break;
				case ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY:

					point.Fe = dfp * point.Fe;
					Fe = point.Fe;
					Fit = glm::transpose(glm::inverse(point.Fe));
					J = glm::determinant(Fe);

					// calculate the polar decomposition F = RU.
					// Then calculate P using F and R
					PolarDecomp(Fe, R, S);

					point.P = 2.0 * mew * (Fe - R) + lam * (J - 1.0) * J * Fit;
					break;
				case ENERGY_MODEL::SIMPLE_SNOW:
					// temporarilly update the Fe
					Fe = dfp * point.Fe;
					Fp = point.Fp;

					F_total = Fe * Fp;

					// calculate the polar decomposition Fe = RU.
					// then use polar decomp to calculate SVD of Fe
					PolarDecomp(Fe, R, S);

					SVD(R, S, U, sig1, sig2, V);

					crit_c = point.crit_c;
					crit_s = point.crit_s;

					sig1 = glm::clamp(sig1, 1.0 - crit_c, 1.0 + crit_s);
					sig2 = glm::clamp(sig2, 1.0 - crit_c, 1.0 + crit_s);

					sigMat = mat2(sig1, 0.0, 0.0, sig2);

					// finally calculate Fe and Fp
					Fe = U * sigMat * transpose(V);
					Fp = V * inverse(sigMat) * transpose(U) * F_total;

					point.Fe = Fe;
					point.Fp = Fp;

					Feit = glm::transpose(glm::inverse(Fe));
					Je = glm::determinant(Fe);
					Jp = glm::determinant(Fp);

					hardening = point.hardening;

					// exp does not work for doubles, so this is a sacrifice we will make for now
					pcof = real(glm::exp(float(hardening * (1.0 - Jp))));

					point.P = 2.0 * mew * (Fe - R) + lam * (Je - 1.0) * Je * Feit;
					point.P *= pcof;
					break;
				default:
					break;
			}
		}
	}
}

void mpm::MpmEngine::GetPointCloudVolumesFromGPUtoCPU(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud)
{
	void* ptr = glMapNamedBuffer(pointCloud->ssbo, GL_READ_ONLY);
	MaterialPoint* data = static_cast<MaterialPoint*>(ptr);

	for (size_t i = 0; i < pointCloud->N; i++) {
		pointCloud->points[i].vol = data[i].vol;
	}

	glUnmapNamedBuffer(pointCloud->ssbo);
}
