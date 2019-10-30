
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
			vec2 mouseForce = m_mousePower * real(m_mouse.w) * glm::normalize(vec2(m_mpm_mouse.x * real(GRID_SIZE_X) - xg.x, m_mpm_mouse.y * real(GRID_SIZE_Y) - xg.y));
			// ignoring (experimental) nodal acceleration
			
			vec2 gridV = nodeMomentum / nodeMass;
			vec2 gridAcc = nodeForce / nodeMass;
			vec2 gridUpdateV = gridV * (1.0 - dt * m_drag) + dt * (gridAcc + m_globalForce + mouseForce);

			m_grid.nodes[index].v = gridUpdateV;

		}
	}
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


			real mew = pointCloudPair.second->parameters.mew;
			real lam = pointCloudPair.second->parameters.lam;

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

					crit_c = pointCloudPair.second->parameters.crit_c;
					crit_s = pointCloudPair.second->parameters.crit_s;

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

					hardening = pointCloudPair.second->parameters.hardening;

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
