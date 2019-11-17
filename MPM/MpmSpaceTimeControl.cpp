#include "MpmSpaceTimeControl.h"

void mpm::control::MPMForwardSimulation(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const vec2 f_ext, const size_t timeSteps, const real dt, bool debugOutput)
{
	for (size_t i = 0; i < timeSteps; i++) {
		if (debugOutput) {
			std::cout << "Time step: " << i << std::endl;
		}
		MPMForwardTimeStep(pointCloud, grid, f_ext, dt);
	}
	std::cout << "Mapping point cloud to GPU...\n";
	pointCloud->MapToGPU();
}

void mpm::control::MPMForwardTimeStep(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const vec2 f_ext, const real dt)
{
	P2G(pointCloud, grid, dt);
	G_Update(grid, f_ext, dt);
	G2P(pointCloud, grid, dt);
}

void mpm::control::P2G(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const real dt)
{
	// reset the grid
	for (size_t i = 0; i < size_t(grid->grid_size_x); i++) {
		for (size_t j = 0; j < size_t(grid->grid_size_y); j++) {
			grid->nodes[i][j].Reset_vpm();
		}
	}

	for (ControlPoint& mp : pointCloud->controlPoints) {


		int botLeftNode_i = int(glm::floor(mp.x.x)) - 1;
		int botLeftNode_j = int(glm::floor(mp.x.y)) - 1;

		mp.P = FixedCorotationalElasticity::PKTensor(mp.F, mp.lam, mp.mew);

		for (int i = 0; i <= 3; i++) {
			for (int j = 0; j <= 3; j++) {

				int currNode_i = botLeftNode_i + i;
				int currNode_j = botLeftNode_j + j;

				if (!InBounds(currNode_i, currNode_j, grid->grid_size_x, grid->grid_size_y)) {
					continue;
				}

				ProjectParticleToGridNode(mp, grid->nodes[currNode_i][currNode_j], dt);
			}
		}
	}
}

void mpm::control::G_Update(std::shared_ptr<ControlGrid> grid, const vec2 f_ext, const real dt)
{
	for (size_t i = 0; i < size_t(grid->grid_size_x); i++) {
		for (size_t j = 0; j < size_t(grid->grid_size_y); j++) {
			if (grid->nodes[i][j].m != 0.0) {
				UpdateGridNode(grid->nodes[i][j], f_ext, dt);
			}
		}
	}
}

void mpm::control::G2P(std::shared_ptr<ControlPointCloud> pointCloud, std::shared_ptr<ControlGrid> grid, const real dt)
{
	for (ControlPoint& mp : pointCloud->controlPoints) {


		int botLeftNode_i = int(glm::floor(mp.x.x)) - 1;
		int botLeftNode_j = int(glm::floor(mp.x.y)) - 1;

		mp.v = vec2(0.0);
		mp.C = mat2(0.0);


		for (int i = 0; i <= 3; i++) {
			for (int j = 0; j <= 3; j++) {

				int currNode_i = botLeftNode_i + i;
				int currNode_j = botLeftNode_j + j;

				if (!InBounds(currNode_i, currNode_j, grid->grid_size_x, grid->grid_size_y)) {
					continue;
				}

				ProjectGridNodeToParticle(grid->nodes[currNode_i][currNode_j], mp, dt);
			}
		}

		UpdateParticle(mp, dt);
	}
}

void mpm::control::ProjectParticleToGridNode(const ControlPoint& mp, ControlGridNode& node, const real dt)
{
	vec2 xg = node.x;
	vec2 xp = mp.x;
	vec2 dgp = xg - xp;
	real wgp = CubicBSpline(dgp.x) * CubicBSpline(dgp.y);

	node.m += wgp * mp.m;
	node.p += wgp * (mp.m * mp.v + (-Dp_inv * dt * mp.vol * mp.P * glm::transpose(mp.F) + mp.m * mp.C) * dgp);
}

void mpm::control::UpdateGridNode(ControlGridNode& node, const vec2 f_ext, const real dt)
{
	// ASSUMING mass != 0

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

void mpm::control::UpdateParticle(ControlPoint& mp, const real dt)
{
	mp.F = (mat2(1.0) + dt * mp.C) * mp.F;
	mp.x += dt * mp.v;
}

void mpm::control::BackPropGridNodeToParticle(const ControlGridNode& node, ControlPoint& mp, const real dt)
{
	vec2 xg = node.x;
	vec2 xp = mp.x;
	vec2 dgp = xg - xp;
	real wgp = CubicBSpline(dgp.x) * CubicBSpline(dgp.y);
}

void mpm::control::ExpandPointCloud(std::shared_ptr<ControlPointCloud> pointCloud, real expansionFactor)
{

	// first find COM;
	vec2 com = vec2(0.0);
	for (size_t i = 0; i < pointCloud->controlPoints.size(); i++) {
		com += pointCloud->controlPoints[i].x;
	}
	com /= real(pointCloud->controlPoints.size());

	// then expand all points around the COM

	for (size_t i = 0; i < pointCloud->controlPoints.size(); i++) {
		pointCloud->controlPoints[i].x -= com;
		pointCloud->controlPoints[i].x *= expansionFactor;
		pointCloud->controlPoints[i].x += com;
	}
}

real mpm::control::PositionLossFunction(std::shared_ptr<const ControlPointCloud> controlPointCloud, std::shared_ptr<const ControlPointCloud> targetPointCloud)
{
	// assuming controlPointCloud and targetPointCloud are the same size
	if (controlPointCloud->controlPoints.size() != targetPointCloud->controlPoints.size()) {
		return -1.0;
	}

	real loss = 0.0;
	for (size_t i = 0; i < controlPointCloud->controlPoints.size(); i++) {

		vec2 d = controlPointCloud->controlPoints[i].x - targetPointCloud->controlPoints[i].x;
		loss += 0.5 * glm::dot(d, d);
	}


	return loss;
}

void mpm::control::OptimizeSetDeformationGradient(std::shared_ptr<ControlPointCloud> controlPointCloud, std::shared_ptr<const ControlPointCloud> targetPointCloud,
														   mat2 initialFe, size_t numTimeSteps, size_t max_iters)
{
	//SaveControlPointCloudOriginalPoints(controlPointCloud);

	//real loss = PositionLossFunction(controlPointCloud, targetPointCloud);
	//std::streamsize prevPrecision = std::cout.precision(16);

	//std::cout << "Initial loss = " << loss << std::endl;


	//for (size_t i = 0; i < max_iters; i++) {
	//	std::cout << "Deformation gradient optimization timestep: " << i << std::endl;
	//	ResetControlPointCloudPointsToSaved(controlPointCloud);
	//	SetDeformationGradientsGLSL(controlPointCloud, initialFe, mat2(1.0), false);
	//	m_mpmAlgorithmEngine->RunMPMSimulationCPP(m_mpmAlgorithmEngine->m_dt, numTimeSteps, false, false);

	//	// Compute loss
	//	loss = PositionLossFunction(controlPointCloud, targetPointCloud);
	//	std::cout << "Loss = " << loss << std::endl;

	//	// Compute gradient


	//	// Descend the gradient
	//}

	//ResetControlPointCloudPointsToSaved(controlPointCloud);
	//std::cout.precision(prevPrecision);

}

void mpm::control::MapCPUControlPointCloudToGPU(std::shared_ptr<ControlPointCloud> pointCloud)
{
	void* ptr = glMapNamedBuffer(pointCloud->ssbo, GL_WRITE_ONLY);
	ControlPoint* data = static_cast<ControlPoint*>(ptr);
	memcpy(data, pointCloud->controlPoints.data(), pointCloud->controlPoints.size() * sizeof(ControlPoint));
	glUnmapNamedBuffer(pointCloud->ssbo);
}

