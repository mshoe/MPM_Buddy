#include "MpmSpaceTimeControl.h"

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

void mpm::control::TranslatePointCloud(std::shared_ptr<ControlPointCloud> pointCloud, vec2 translation)
{
	for (size_t i = 0; i < pointCloud->controlPoints.size(); i++) {
		pointCloud->controlPoints[i].x += translation;
	}
}

void mpm::control::TransformPointCloud(std::shared_ptr<ControlPointCloud> pointCloud, mat2 transformMat)
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
		pointCloud->controlPoints[i].x = transformMat * pointCloud->controlPoints[i].x;
		pointCloud->controlPoints[i].x += com;
	}
}

real mpm::control::ComputeLoss(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg, 
							   LOSS_FUNCTION lossFunction, 
							   const mp_penalty_vector& penalties,
							   const real drag, const real dt)
{
	real loss = -1.0;

	switch (lossFunction) {
	case LOSS_FUNCTION::PARTICLE_POSITIONS:
		loss = ParticlePositionLossFunction(stcg->simStates.back()->pointCloud, stcg->targetPointCloud);
		break;
	case LOSS_FUNCTION::GRID_NODE_MASSES:
		loss = GridMassLossFunction_WithPenalty(stcg->simStates.back()->pointCloud,
												stcg->simStates.back()->grid,
												stcg->targetGrid,
												penalties,
												drag,
												dt);
		break;
	default:
		std::cout << "error" << std::endl;
		break;
	}
	return loss;
}

real mpm::control::ParticlePositionLossFunction(std::shared_ptr<const ControlPointCloud> controlPointCloud, std::shared_ptr<const ControlPointCloud> targetPointCloud)
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

//real mpm::control::GridMassLossFunction(std::shared_ptr<ControlPointCloud> controlPointCloud, 
//										std::shared_ptr<ControlGrid> controlGrid, 
//										std::shared_ptr<const TargetGrid> targetGrid,
//										const real dt)
//{
//
//	if (controlGrid->grid_size_x != targetGrid->grid_size_x ||
//		controlGrid->grid_size_y != targetGrid->grid_size_y) {
//		return -1.0;
//	}
//
//	P2G(controlPointCloud, controlGrid, dt);
//
//	real loss = 0.0;
//	for (int i = 0; i < controlGrid->grid_size_x; i++) {
//		for (int j = 0; j < controlGrid->grid_size_y; j++) {
//			real dm = controlGrid->Node(i, j).m - targetGrid->ConstNode(i, j).m;
//			loss += 0.5 * dm * dm;
//		}
//	}
//	return loss;
//}

real mpm::control::GridMassLossFunction_WithPenalty(std::shared_ptr<ControlPointCloud> controlPointCloud, 
													std::shared_ptr<ControlGrid> controlGrid, 
													std::shared_ptr<const TargetGrid> targetGrid, 
													const mp_penalty_vector& penalties,
													const real drag, const real dt)
{
	if (controlGrid->grid_size_x != targetGrid->grid_size_x ||
		controlGrid->grid_size_y != targetGrid->grid_size_y) {
		return -1.0;
	}

	P2G(controlPointCloud, controlGrid, drag, dt);

	real loss = 0.0;
	for (int i = 0; i < controlGrid->grid_size_x; i++) {
		for (int j = 0; j < controlGrid->grid_size_y; j++) {
			real dm = controlGrid->Node(i, j).m - targetGrid->ConstNode(i, j).m;
			loss += 0.5 * dm * dm * targetGrid->penaltyWeights[i][j];
		}
	}

	for (size_t i = 0; i < penalties.size(); i++) {
		switch (penalties[i].first) {
		case LOSS_PENALTY::MP_VELOCITIES:
			loss += MpVelocityPenalty(controlPointCloud) * penalties[i].second;
			break;
		case LOSS_PENALTY::MP_DEFGRADS:
			loss += MpDefGradPenalty(controlPointCloud) * penalties[i].second;
			break;
		default:
			break;
		}
	}

	return loss;
}

real mpm::control::MpDefGradPenalty(std::shared_ptr<ControlPointCloud> controlPointCloud)
{

	real loss = 0.0;
	for (int p = 0; p < controlPointCloud->controlPoints.size(); p++) {
		loss += MatrixNormSqrd(controlPointCloud->controlPoints[p].F);
	}
	return loss;
}

real mpm::control::MpVelocityPenalty(std::shared_ptr<ControlPointCloud> controlPointCloud)
{
	real loss = 0.0;
	for (int p = 0; p < controlPointCloud->controlPoints.size(); p++) {
		real vx = controlPointCloud->controlPoints[p].v.x;
		real vy = controlPointCloud->controlPoints[p].v.y;
		loss += 0.5 * (vx * vx + vy * vy);
	}
	return loss;
}

void mpm::control::OptimizeSetDeformationGradient_InTemporalOrder(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg,
																  const vec2 f_ext, const real drag, const real dt,
																  bool setInitialFe, mat2 initialFe,
																  int optFrameOffset,
																  int numTimeSteps, int max_iters, int maxLineSearchIters,
																  int totalTemporalIterations,
																  LOSS_FUNCTION lossFunction, 
																  const mp_penalty_vector& penalties,
																  bool forceDescent,
																  bool reverseTime, real penalty,
																  real initialFAlpha, real initialMaterialAlpha,
																  real tol, real suffTemporalIterLossDecreaseFactor,
																  bool optimizeOnlyInitialF, bool debugOutput)
{
	switch (lossFunction) {
	case LOSS_FUNCTION::PARTICLE_POSITIONS:
		if (stcg->targetPointCloud == nullptr ||
			stcg->controlPointCloud == nullptr ||
			stcg->targetPointCloud->controlPoints.size() !=
			stcg->controlPointCloud->controlPoints.size()) {
			std::cout << "error" << std::endl;
		}
		break;
	case LOSS_FUNCTION::GRID_NODE_MASSES:
		// create the target grid from the target point cloud
		stcg->targetGrid = std::make_shared<TargetGrid>(stcg->grid_size_x, stcg->grid_size_y);
		P2G(stcg->targetPointCloud, stcg->targetGrid, drag, dt);
		MapCPUControlGridToGPU(stcg->targetGrid, stcg->targetGridSsbo);
		stcg->targetGrid->InitializePenaltyWeights(penalty);
		break;
	default:
		std::cout << "error" << std::endl;
		return;
	}


	//SaveControlPointCloudOriginalPoints(controlPointCloud);
	stcg->originalPointCloud = std::make_shared<ControlPointCloud>(stcg->controlPointCloud);
	stcg->originalPointCloud->ResetdFc(); // we optimize these with a 0 initial guess



	if (numTimeSteps < 1) {
		std::cout << "error: numTimeSteps < 1" << std::endl;
		return;
	}

	stcg->timeSteps = numTimeSteps;
	stcg->InitSTCG();

	real loss;
	std::streamsize prevPrecision = std::cout.precision(16);


	if (setInitialFe) {
		mat2 controlF = initialFe;
		stcg->simStates[0]->pointCloud->SetF(controlF);
	}
	real alpha = initialFAlpha;

	stcg->lossValues.clear();

	// for descending only 1 point at a time
	//ControlPoint dummy_mp;
	//ControlPoint& max_mp = dummy_mp;

	bool timeStepOptFLossDecreased = false;

	int initialTimeStep;
	int timeIncrement;
	if (reverseTime) {
		initialTimeStep = int(stcg->simStates.size()) - 2;
		timeIncrement = -optFrameOffset;
	}
	else {
		initialTimeStep = 0;
		timeIncrement = optFrameOffset;
	}

	for (int temporalIter = 0; temporalIter < totalTemporalIterations; temporalIter++) {
		std::cout << "Temporal iteration: " << temporalIter << std::endl;

		bool temporalIterationConverged = true;
		bool lossDecreased = false;


		MPMForwardSimulation(stcg, f_ext, drag, dt, 0, debugOutput);
		real temporalIterLoss = ComputeLoss(stcg, lossFunction, penalties, drag, dt);
		std::cout << "Temporal iter " << temporalIter << " initial loss = " << temporalIterLoss << std::endl;

		for (int timeStep = initialTimeStep; 0 <= timeStep && timeStep < int(stcg->simStates.size()) - 1; timeStep += timeIncrement) {
			std::cout << "Optimizing for timestep: " << timeStep << std::endl;
			
			bool timeStepConverged = false;
			

			std::cout << "Optimizing F" << std::endl;
			alpha = initialFAlpha;
			for (int i = 0; i < max_iters; i++) {
				std::cout << "Gradient descent iteration: " << i << std::endl;


				MPMForwardSimulation(stcg, f_ext, drag, dt, timeStep, debugOutput);


				// Compute loss
				loss = ComputeLoss(stcg, lossFunction, penalties, drag, dt);
				std::cout << "Loss = " << loss << std::endl;

				// IMGUI ONLY WORKS WITH FLOATS HERE
				// assuming loss will pretty much always be > 0
				stcg->lossValues.push_back(float(log(loss)));

				// Compute gradients
				MPMBackPropogation(stcg, drag, dt, lossFunction, timeStep, penalties, debugOutput);

				if (stcg->simStates[timeStep]->pointCloud->Check_dLdF_Nan()) {
					std::cout << "Nan dLdF found..." << std::endl;
					break;
				}

				// only check convergence on the initial iteration
				if (i == 0) {
					timeStepConverged = stcg->simStates[timeStep]->pointCloud->CheckFConvergence(tol, true);
				}

				if (timeStepConverged) {
					std::cout << "Time step is converged." << std::endl;
					break;
				}

				bool lineSearchLossDecreased = false;
				for (int iter = 0; iter < maxLineSearchIters; iter++) {

					stcg->simStates[timeStep]->pointCloud->DescendFGradients(alpha);
					//stcg->simStates[timeStep]->pointCloud->DescendMaterialAndFGradients(alpha);

					MPMForwardSimulation(stcg, f_ext, drag, dt, timeStep, debugOutput);
					real nextLoss = ComputeLoss(stcg, lossFunction, penalties, drag, dt);
					std::cout << "Line search iter: " << iter << ", loss = " << nextLoss << std::endl;
					// if the loss fxn decreased, then we r good
					if (nextLoss < loss) {
						lineSearchLossDecreased = true;
						lossDecreased = true;
						break;
					}

					
					stcg->simStates[timeStep]->pointCloud->DescendFGradients(-alpha);
					
					
					// TODO: Print dFc values here to make sure they are going back to previous value


					alpha /= 2.0;
				}

				

				if (!lineSearchLossDecreased) {
					std::cout << "Line search unable to find a loss decrease this time step..." << std::endl;


					// Recompute this here. This is important to make sure the computation graph values all make sense
					MPMForwardSimulation(stcg, f_ext, drag, dt, timeStep, debugOutput);
					real finalLossThisTimeStep = ComputeLoss(stcg, lossFunction, penalties, drag, dt);
					std::cout << "Loss is still: " << finalLossThisTimeStep << "..." << std::endl;

					break; // go to next time step
				}
			}

			temporalIterationConverged = temporalIterationConverged && timeStepConverged;
		}

		
		if (!lossDecreased) {
			std::cout << "Failed to find a decrease..." << std::endl;
			break;
		}

		if (temporalIterationConverged) {
			std::cout << "Gradients below tolerance..." << std::endl;
			break;
		}

		MPMForwardSimulation(stcg, f_ext, drag, dt, 0, debugOutput);
		real temporalIterFinalLoss = ComputeLoss(stcg, lossFunction, penalties, drag, dt);
		std::cout << "Temporal iter " << temporalIter << " initial loss = " << temporalIterLoss << std::endl;
		std::cout << "Temporal iter " << temporalIter << " final loss = " << temporalIterFinalLoss << std::endl;
		std::cout << "Maximum loss for a sufficient decrease: " << temporalIterLoss * (1.0 - suffTemporalIterLossDecreaseFactor) << std::endl;

		if (temporalIterFinalLoss > temporalIterLoss* (1.0 - suffTemporalIterLossDecreaseFactor)) {
			std::cout << "A sufficient decrease in the loss function was not found... ending optimization" << std::endl;
			break;
		}
	}

	//stcg->outputPointCloud = std::make_shared<ControlPointCloud>(stcg->simStates[0]->pointCloud);

	std::cout.precision(prevPrecision);
}

void mpm::control::GenControlPointCloudSSBO(std::shared_ptr<ControlPointCloud> pointCloud, GLuint& ssbo)
{
	glCreateBuffers(1, &ssbo);
	glNamedBufferStorage(
		ssbo,
		sizeof(ControlPoint) * pointCloud->controlPoints.size(),
		pointCloud->controlPoints.data(),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // add write bit for cpu mode
	);
}

void mpm::control::GenControlGridSSBO(std::shared_ptr<ControlGrid> grid, GLuint& ssbo)
{
	glCreateBuffers(1, &ssbo);

	glNamedBufferStorage(
		ssbo,
		sizeof(ControlGridNode) * grid->grid_size_x * grid->grid_size_y,
		grid->nodes.data(),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // add write bit for cpu mode
	);
}

void mpm::control::MapCPUControlPointCloudToGPU(std::shared_ptr<ControlPointCloud> pointCloud, GLuint ssbo)
{
	void* ptr = glMapNamedBuffer(ssbo, GL_WRITE_ONLY);
	ControlPoint* data = static_cast<ControlPoint*>(ptr);
	memcpy(data, pointCloud->controlPoints.data(), pointCloud->controlPoints.size() * sizeof(ControlPoint));
	glUnmapNamedBuffer(ssbo);
}

void mpm::control::MapCPUControlGridToGPU(std::shared_ptr<ControlGrid> grid, GLuint ssbo)
{
	void* ptr = glMapNamedBuffer(ssbo, GL_WRITE_ONLY);
	ControlGridNode* data = static_cast<ControlGridNode*>(ptr);
	
	for (size_t i = 0; i < grid->grid_size_x; i++) {

		// grid ssbo is always gonna be GRID_SIZE_X * GRID_SIZE_Y
		memcpy(data + i * GRID_SIZE_Y, &grid->nodes[0] + i * grid->grid_size_y, grid->grid_size_y * sizeof(ControlGridNode));
	}
	glUnmapNamedBuffer(ssbo);
}



//void mpm::control::MPMSpaceTimeComputationGraph::OptimizeControlF()
//{
//
//	InitSTCG();
//
//	std::cout << sizeof(ControlPoint) * originalPointCloud->controlPoints.size() << std::endl;
//	std::cout << sizeof(ControlGridNode) * grid_size_x * grid_size_y << std::endl;
//
//	for (int iter = 0; iter < iters; iter++) {
//
//	}
//
//}


