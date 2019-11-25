#include "MpmSpaceTimeControl.h"

void mpm::control::MPMBackPropogation(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg, 
									  const real dt, 
									  LOSS_FUNCTION lossFunction, 
									  int controlTimeStep,
									  bool debugOutput)
{
	switch (lossFunction) {
	case LOSS_FUNCTION::PARTICLE_POSITIONS:
		BackPropPointCloudPositionLossFunctionInit(stcg->simStates.back()->pointCloud,
												   stcg->targetPointCloud,
												   dt);
		break;
	case LOSS_FUNCTION::GRID_NODE_MASSES:
		BackPropGridMassLossFunctionInit(stcg->simStates.back()->pointCloud,
										 stcg->simStates.back()->grid,
										 stcg->targetGrid,
										 dt);
		break;
	default:
		std::cout << "error" << std::endl;
		return;
	}

	

	if (stcg->simStates.empty()) {

		std::cout << "error, simStates size is 0\n";
		return;
	}

	// Only back-propogate to the time step we want
	for (int i = stcg->simStates.size() - 2; i >= controlTimeStep; i--) {

		if (debugOutput) {
			std::cout << "Time step: " << i << std::endl;
		}

		stcg->simStates[i]->pointCloud->ResetGradients();
		stcg->simStates[i]->grid->ResetGradients();
		MPMBackPropogationTimeStep(stcg->simStates[i + 1], stcg->simStates[i], dt);
	}
}

void mpm::control::BackPropPointCloudPositionLossFunctionInit(std::shared_ptr<ControlPointCloud> controlPointCloud, std::shared_ptr<const ControlPointCloud> targetPointCloud, const real dt)
{

	if (targetPointCloud->controlPoints.size() !=
		controlPointCloud->controlPoints.size()) 
	{
		std::cout << "error, point cloud sizes don't match\n";
		return;
	}

	// Initialize back prop
	for (size_t p = 0; p < targetPointCloud->controlPoints.size(); p++) {
		BackPropParticlePositionLossFunctionInit(controlPointCloud->controlPoints[p],
												 targetPointCloud->controlPoints[p],
												 dt);
	}
}

void mpm::control::BackPropParticlePositionLossFunctionInit(ControlPoint& mp, const ControlPoint& mp_target, const real dt)
{
	// this assumes mp is the final position of the particle

	// dL / dx_n is just the derivative of the loss fxn w.r.t x_n
	mp.dLdx = mp.x - mp_target.x;

	// dLdF is 0, since this mp's F will not affect its current position in this time step,
	// it can only affect the next position
	mp.dLdF = mat2(0.0);

	// These 2 gradients are enough to start off the backpropogation:
	// dL / dv
	mp.dLdv = dt * mp.dLdx;

	// dL / dC
	mp.dLdC = mat2(0.0);
}

void mpm::control::BackPropGridMassLossFunctionInit(std::shared_ptr<ControlPointCloud> controlPointCloud, 
													std::shared_ptr<ControlGrid> controlGrid, 
													std::shared_ptr<const ControlGrid> targetGrid, 
													const real dt)
{
	if (controlGrid->grid_size_x != targetGrid->grid_size_x ||
		controlGrid->grid_size_y != targetGrid->grid_size_y) {
		std::cout << "error: grids not same size" << std::endl;
	}

	// first get masses onto grid
	P2G(controlPointCloud, controlGrid, dt);

	// compute dL / dm per each node
	for (int i = 0; i < controlGrid->grid_size_x; i++) {
		for (int j = 0; j < controlGrid->grid_size_y; j++) {
			controlGrid->nodes[i][j].dLdm = controlGrid->nodes[i][j].m - targetGrid->nodes[i][j].m;
			//std::cout << i << ", " << j << ": " << controlGrid->nodes[i][j].dLdm << "| ";
		}
		//std::cout << std::endl;
	}
	
	// then compute initial dL / dx_p per particle
	for (size_t p = 0; p < controlPointCloud->controlPoints.size(); p++) {
		ControlPoint& mp = controlPointCloud->controlPoints[p];
		
		// Make sure this is initialized to 0
		mp.dLdx = vec2(0.0);

		int botLeftNode_i = int(glm::floor(mp.x.x)) - 1;
		int botLeftNode_j = int(glm::floor(mp.x.y)) - 1;

		for (int i = 0; i <= 3; i++) {
			for (int j = 0; j <= 3; j++) {

				int currNode_i = botLeftNode_i + i;
				int currNode_j = botLeftNode_j + j;

				if (!InBounds(currNode_i, currNode_j, controlGrid->grid_size_x, controlGrid->grid_size_y)) {
					continue;
				}

				const ControlGridNode& gn = controlGrid->nodes[currNode_i][currNode_j];

				vec2 xg = gn.x;
				vec2 xp = mp.x;
				vec2 dgp = xg - xp;
				real wgp = CubicBSpline(dgp.x) * CubicBSpline(dgp.y);
				vec2 wpg_Grad = -vec2(CubicBSplineSlope(dgp.x) * CubicBSpline(dgp.y),
									 CubicBSpline(dgp.x) * CubicBSplineSlope(dgp.y));

				mp.dLdx += mp.m * gn.dLdm * wpg_Grad;
				// compute initial dL / dx_p. 
				// This only has a contribution from dL / dm at this time step
			}
		}
	}


	// then compute other initial gradients
	for (size_t p = 0; p < controlPointCloud->controlPoints.size(); p++) {
		ControlPoint& mp = controlPointCloud->controlPoints[p];

		// dLdF is 0, since this mp's F will not affect its current position in this time step,
		// it can only affect the next position
		mp.dLdF = mat2(0.0);

		// These 2 gradients are enough to start off the backpropogation:
		// dL / dv
		mp.dLdv = dt * mp.dLdx;

		// dL / dC
		mp.dLdC = mat2(0.0);
	}
	
}

void mpm::control::MPMBackPropogationTimeStep(std::shared_ptr<MPMSpaceComputationGraph> scg_nplus1,
											  std::shared_ptr<MPMSpaceComputationGraph> scg_n,
											  const real dt)
{

	BackG2P(scg_nplus1->pointCloud, scg_n->pointCloud, scg_n->grid, dt);
	BackG_Update(scg_n->grid);
	BackP2G(scg_nplus1->pointCloud, scg_n->pointCloud, scg_n->grid, dt);
}



void mpm::control::BackG2P(std::shared_ptr<const ControlPointCloud> pointCloud_nplus1,
						   std::shared_ptr<const ControlPointCloud> pointCloud_n,
						   std::shared_ptr<ControlGrid> grid, const real dt)
{

	for (size_t p = 0; p < pointCloud_n->controlPoints.size(); p++) {
		const ControlPoint& mp_nplus1 = pointCloud_nplus1->controlPoints[p];
		const ControlPoint& mp_n = pointCloud_n->controlPoints[p];


		int botLeftNode_i = int(glm::floor(mp_n.x.x)) - 1;
		int botLeftNode_j = int(glm::floor(mp_n.x.y)) - 1;

		for (int i = 0; i <= 3; i++) {
			for (int j = 0; j <= 3; j++) {

				int currNode_i = botLeftNode_i + i;
				int currNode_j = botLeftNode_j + j;

				if (!InBounds(currNode_i, currNode_j, grid->grid_size_x, grid->grid_size_y)) {
					continue;
				}

				BackPropParticleToGridNode(mp_nplus1, mp_n, grid->nodes[currNode_i][currNode_j], dt);
			}
		}
	}

}

void mpm::control::BackG_Update(std::shared_ptr<ControlGrid> grid)
{

	for (size_t i = 0; i < size_t(grid->grid_size_x); i++) {
		for (size_t j = 0; j < size_t(grid->grid_size_y); j++) {
			if (grid->nodes[i][j].m != 0.0) {
				BackPropGridNode(grid->nodes[i][j]);
			}
		}
	}

}

void mpm::control::BackP2G(std::shared_ptr<const ControlPointCloud> pointCloud_nplus1, std::shared_ptr<ControlPointCloud> pointCloud_n, std::shared_ptr<const ControlGrid> grid, const real dt)
{
	for (size_t p = 0; p < pointCloud_n->controlPoints.size(); p++) {
		const ControlPoint& mp_nplus1 = pointCloud_nplus1->controlPoints[p];
		ControlPoint& mp_n = pointCloud_n->controlPoints[p];

		int botLeftNode_i = int(glm::floor(mp_n.x.x)) - 1;
		int botLeftNode_j = int(glm::floor(mp_n.x.y)) - 1;

		for (int i = 0; i <= 3; i++) {
			for (int j = 0; j <= 3; j++) {

				int currNode_i = botLeftNode_i + i;
				int currNode_j = botLeftNode_j + j;

				if (!InBounds(currNode_i, currNode_j, grid->grid_size_x, grid->grid_size_y)) {
					continue;
				}

				BackPropGridNodeToParticle(grid->nodes[currNode_i][currNode_j], mp_n, dt);
			}
		}

		BackPropParticleToParticle(mp_nplus1, mp_n, dt);
	}
}

void mpm::control::BackPropParticleToGridNode(const ControlPoint& mp, const ControlPoint& mp_prev, ControlGridNode& node, const real dt)
{
	vec2 xg = node.x;
	vec2 xp = mp_prev.x;
	vec2 dgp = xg - xp;
	real wgp = CubicBSpline(dgp.x) * CubicBSpline(dgp.y);
	node.dLdv += mp.dLdv * wgp + Dp_inv * wgp * mp.dLdC * dgp;
}

void mpm::control::BackPropGridNode(ControlGridNode& node)
{
	node.dLdp = node.dLdv / node.m;

	node.dLdm = -1.0 / node.m * glm::dot(node.v, node.dLdv);
}

void mpm::control::BackPropGridNodeToParticle(const ControlGridNode& node, ControlPoint& mp_prev, const real dt)
{
	vec2 xg = node.x;
	vec2 xp = mp_prev.x;
	vec2 dgp = xg - xp;
	real wgp = CubicBSpline(dgp.x) * CubicBSpline(dgp.y);

	// dL / dv
	// Not necessary to compute here b/c we compute it in BackPropParticle right?
	mp_prev.dLdv += wgp * mp_prev.m * node.dLdp;

	// dL / dP
	mp_prev.dLdP -= wgp * Dp_inv * dt * mp_prev.vol * glm::outerProduct(node.dLdp, glm::transpose(mp_prev.F + mp_prev.dFc) * dgp);

	// dL / dF
	// Note: this gradient also gets particle contributions, which will be added later
	mp_prev.dLdF -= wgp * Dp_inv * dt * mp_prev.vol * glm::outerProduct(dgp, node.dLdp * mp_prev.P);

	// dL / dC
	// Not necessary to compute here b/c we compute it in BackPropParticle right
	mp_prev.dLdC += wgp * mp_prev.m * glm::outerProduct(node.dLdp, dgp);

	// dL / dx
	// Note: this gradient also gets particle contributions, which will be added later
	//mp_prev.dLdx += 
}

void mpm::control::BackPropParticleToParticle(const ControlPoint& mp, ControlPoint& mp_prev, const real dt)
{
	mp_prev.dLdF += glm::transpose(mat2(1.0) + dt * mp.dLdC) * mp.dLdF;
	mp_prev.dLdF += FixedCorotationalElasticity::d2Psi_dF2_multbydF(mp_prev.F + mp_prev.dFc, mp_prev.lam, mp_prev.mew, mp_prev.dLdP);

}