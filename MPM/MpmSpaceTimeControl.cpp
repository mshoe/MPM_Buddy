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

void mpm::control::OptimizeSetDeformationGradient(std::shared_ptr<MPMSpaceTimeComputationGraph> stcg,
												  const vec2 f_ext, const real dt,
												  mat2 initialFe, int optFrameOffset,
												  int numTimeSteps, int max_iters, int maxLineSearchIters,
												  real initialAlpha, bool optimizeOnlyInitialF, bool debugOutput)
{
	//SaveControlPointCloudOriginalPoints(controlPointCloud);
	stcg->originalPointCloud = std::make_shared<ControlPointCloud>(stcg->controlPointCloud);
	stcg->originalPointCloud->ResetdFc(); // we optimize these with a 0 initial guess

	real loss = PositionLossFunction(stcg->originalPointCloud, stcg->targetPointCloud);
	std::streamsize prevPrecision = std::cout.precision(16);

	std::cout << "Initial loss = " << loss << std::endl;

	if (numTimeSteps < 1) {
		std::cout << "error: numTimeSteps < 1" << std::endl;
		return;
	}

	stcg->timeSteps = numTimeSteps;
	stcg->InitSTCG();


	mat2 controlF = initialFe;
	stcg->simStates[0]->pointCloud->SetF(controlF);
	real alpha = initialAlpha;

	for (int i = 0; i < max_iters; i++) {
		std::cout << "Gradient descent iteration: " << i << std::endl;

		MPMForwardSimulation(stcg, f_ext, dt, debugOutput);


		// Compute loss
		loss = PositionLossFunction(stcg->simStates[numTimeSteps-1]->pointCloud, 
									stcg->targetPointCloud);
		std::cout << "Loss = " << loss << std::endl;

		// Compute gradients

		MPMBackPropogation(stcg, dt, debugOutput);

		if (optimizeOnlyInitialF) {
			// Descend the gradients via line search
			bool lossDecreased = false;
			for (int iter = 0; iter < maxLineSearchIters; iter++) {

				stcg->simStates[0]->pointCloud->DescendFGradients(alpha);

				MPMForwardSimulation(stcg, f_ext, dt, debugOutput);
				real nextLoss = PositionLossFunction(stcg->simStates[numTimeSteps - 1]->pointCloud,
														stcg->targetPointCloud);
				std::cout << "Line search iter: " << iter << ", loss = " << nextLoss << std::endl;
				// if the loss fxn decreased, then we r good
				if (nextLoss < loss) {
					lossDecreased = true;
					break;
				}

				stcg->simStates[0]->pointCloud->DescendFGradients(-alpha);
				alpha /= 2.0;
			}
			

			if (!lossDecreased) {
				std::cout << "Line search failed to find decrease. Terminating..." << std::endl;
				break;
			}
		}
		else {
			// Descend the gradients via line search
			bool lossDecreased = false;
			for (int iter = 0; iter < maxLineSearchIters; iter++) {
				std::cout << "Line search iter: " << iter << std::endl;
				// Find the timestep that you want to descend the gradients for
				bool timeStepOptFLossDecreased = false;
				for (int timeStep = 0; timeStep < int(stcg->simStates.size()) - 1; timeStep += optFrameOffset) {
					stcg->simStates[timeStep]->pointCloud->DescendFGradients(alpha);

					MPMForwardSimulation(stcg, f_ext, dt, debugOutput);
					real nextLoss = PositionLossFunction(stcg->simStates[numTimeSteps - 1]->pointCloud,
														 stcg->targetPointCloud);
					// if the loss fxn decreased, then we r good
					if (nextLoss < loss) {
						std::cout << "Found loss decrease direction in timestep: " << timeStep << std::endl;
						std::cout << "nextLoss = " << nextLoss << std::endl;
						timeStepOptFLossDecreased = true;
						break;
					}

					stcg->simStates[timeStep]->pointCloud->DescendFGradients(-alpha);
				}

				if (!timeStepOptFLossDecreased) {
					alpha /= 2.0;
				}
				else {
					lossDecreased = true;
					break;
				}
			}

			if (!lossDecreased) {
				std::cout << "Line search failed to find decrease. Terminating..." << std::endl;
				break;
			}
		}
	}

	stcg->outputPointCloud = std::make_shared<ControlPointCloud>(stcg->simStates[0]->pointCloud);

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

void mpm::control::MapCPUControlPointCloudToGPU(std::shared_ptr<ControlPointCloud> pointCloud, GLuint ssbo)
{
	void* ptr = glMapNamedBuffer(ssbo, GL_WRITE_ONLY);
	ControlPoint* data = static_cast<ControlPoint*>(ptr);
	memcpy(data, pointCloud->controlPoints.data(), pointCloud->controlPoints.size() * sizeof(ControlPoint));
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

