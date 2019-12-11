#include "MpmSpaceTimeControl.h"

void mpm::control::MPMForwardSimulation(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg, 
										const vec2 f_ext, const real dt, 
										int controlTimeStep, 
										bool debugOutput)
{
	// stcg: spacetime computation graph

	for (size_t i = controlTimeStep; i < stcg->simStates.size() - 1; i++) {
		if (debugOutput) {
			std::cout << "Time step: " << i << std::endl;
		}

		// copy the point cloud from the previous time step
		stcg->simStates[i + 1]->pointCloud->SetFromPreviousTimeStepControlPointCloud(stcg->simStates[i]->pointCloud);

		MPMForwardTimeStep(
			stcg->simStates[i],
			stcg->simStates[i + 1],
			f_ext, dt);

	}
	if (debugOutput) {
		std::cout << "Mapping point cloud to GPU...\n";
		MapCPUControlPointCloudToGPU(stcg->simStates[stcg->simStates.size() - 1]->pointCloud, stcg->controlSsbo);
	}
}

void mpm::control::MPMForwardTimeStep(std::shared_ptr<MPMSpaceComputationGraph> scg_n,
									  std::shared_ptr<MPMSpaceComputationGraph> scg_nplus1,
									  const vec2 f_ext, const real dt)
{
	P2G(scg_n->pointCloud, scg_n->grid, dt);
	G_Update(scg_n->grid, f_ext, dt);
	G2P(scg_n->pointCloud, scg_nplus1->pointCloud, scg_n->grid, dt);
}

void mpm::control::P2G(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const real dt)
{
	// reset the grid
	for (size_t i = 0; i < size_t(grid->grid_size_x); i++) {
		for (size_t j = 0; j < size_t(grid->grid_size_y); j++) {
			grid->Node(i, j).Reset_vpm();
		}
	}

	for (size_t p = 0; p < pointCloud->controlPoints.size(); p++) {
		ControlPoint& mp = pointCloud->controlPoints[p];

		int botLeftNode_i = int(glm::floor(mp.x.x)) - 1;
		int botLeftNode_j = int(glm::floor(mp.x.y)) - 1;

		mp.P = FixedCorotationalElasticity::PKTensor(mp.F + mp.dFc, mp.lam + mp.dlamc, mp.mew + mp.dmewc);

		for (int i = 0; i <= 3; i++) {
			for (int j = 0; j <= 3; j++) {

				int currNode_i = botLeftNode_i + i;
				int currNode_j = botLeftNode_j + j;

				if (!InBounds(currNode_i, currNode_j, grid->grid_size_x, grid->grid_size_y)) {
					continue;
				}

				ProjectParticleToGridNode(mp, grid->Node(currNode_i, currNode_j), dt);
			}
		}
	}
}

void mpm::control::G_Update(std::shared_ptr<ControlGrid> grid, const vec2 f_ext, const real dt)
{
	for (size_t i = 0; i < size_t(grid->grid_size_x); i++) {
		for (size_t j = 0; j < size_t(grid->grid_size_y); j++) {
			if (grid->Node(i, j).m != 0.0) {
				UpdateGridNode(grid->Node(i, j), f_ext, dt);
			}
		}
	}
}

void mpm::control::G2P(std::shared_ptr<const ControlPointCloud> pointCloud_n, 
					   std::shared_ptr<ControlPointCloud> pointCloud_nplus1, 
					   std::shared_ptr<const ControlGrid> grid, const real dt)
{
	for (size_t p = 0; p < pointCloud_n->controlPoints.size(); p++) {
		ControlPoint& mp_nplus1 = pointCloud_nplus1->controlPoints[p];
		const ControlPoint& mp_n = pointCloud_n->controlPoints[p];

		int botLeftNode_i = int(glm::floor(mp_n.x.x)) - 1;
		int botLeftNode_j = int(glm::floor(mp_n.x.y)) - 1;

		mp_nplus1.v = vec2(0.0);
		mp_nplus1.C = mat2(0.0);


		for (int i = 0; i <= 3; i++) {
			for (int j = 0; j <= 3; j++) {

				int currNode_i = botLeftNode_i + i;
				int currNode_j = botLeftNode_j + j;

				if (!InBounds(currNode_i, currNode_j, grid->grid_size_x, grid->grid_size_y)) {
					continue;
				}

				ProjectGridNodeToParticle(grid->ConstNode(currNode_i, currNode_j), mp_nplus1, dt);
			}
		}

		UpdateParticle(mp_nplus1, mp_n, dt);
	}
}

void mpm::control::ProjectParticleToGridNode(const ControlPoint& mp, ControlGridNode& node, const real dt)
{
	vec2 xg = node.x;
	vec2 xp = mp.x;
	vec2 dgp = xg - xp;
	real wgp = CubicBSpline(dgp.x) * CubicBSpline(dgp.y);

	node.m += wgp * mp.m;
	node.p += wgp * (mp.m * mp.v + (-Dp_inv * dt * mp.vol * mp.P * glm::transpose(mp.F + mp.dFc) + mp.m * mp.C) * dgp);
}

void mpm::control::UpdateGridNode(ControlGridNode& node, const vec2 f_ext, const real dt)
{
	// ASSUMING mass != 0

	// should I implement a drag force??

	node.v = node.p / node.m + dt * f_ext;
}

void mpm::control::ProjectGridNodeToParticle(const ControlGridNode& node, ControlPoint& mp, const real dt)
{
	vec2 xg = node.x;
	vec2 xp = mp.x;
	vec2 dgp = xg - xp;
	real wgp = CubicBSpline(dgp.x) * CubicBSpline(dgp.y);

	mp.v += wgp * node.v;
	mp.C += Dp_inv * wgp * glm::outerProduct(node.v, dgp);
}

void mpm::control::UpdateParticle(ControlPoint& mp, const ControlPoint& mp_prev, const real dt)
{
	mp.F = (mat2(1.0) + dt * mp.C) * (mp_prev.F + mp_prev.dFc);
	mp.x += dt * mp.v;
	mp.lam = mp_prev.lam + mp_prev.dLdlam;
	mp.mew = mp_prev.mew + mp_prev.dLdmew;
}