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
												  mat2 initialFe, int numTimeSteps, int max_iters,
												  real initialAlpha, bool debugOutput)
{
	//SaveControlPointCloudOriginalPoints(controlPointCloud);
	stcg->originalPointCloud = std::make_shared<ControlPointCloud>(stcg->controlPointCloud);


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

	for (int i = 0; i < max_iters; i++) {
		std::cout << "Deformation gradient optimization timestep: " << i << std::endl;

		//stcg->simStates[0]->pointCloud->SetFromControlPointCloud(stcg->originalPointCloud);
		

		

		/*ResetControlPointCloudPointsToSaved(controlPointCloud);
		SetDeformationGradientsGLSL(controlPointCloud, initialFe, mat2(1.0), false);
		m_mpmAlgorithmEngine->RunMPMSimulationCPP(m_mpmAlgorithmEngine->m_dt, numTimeSteps, false, false);*/

		MPMForwardSimulation(stcg, f_ext, dt, debugOutput);


		// Compute loss
		loss = PositionLossFunction(stcg->simStates[numTimeSteps-1]->pointCloud, 
									stcg->targetPointCloud);
		std::cout << "Loss = " << loss << std::endl;

		// Compute gradients

		MPMBackPropogation(stcg, dt, debugOutput);

		// Descend the gradients
		for (ControlPoint& mp : stcg->simStates[0]->pointCloud->controlPoints) {

			//std::cout << glm::to_string(mp.dLdF) << std::endl << std::endl;

			// view this dLdF matrix as a vector
			// descend in the direction of the vector
			mat2 dLdF_dir = NormalizedMatrix(mp.dLdF);

			mp.F -= initialAlpha * dLdF_dir;
		}
	}

	stcg->outputPointCloud = std::make_shared<ControlPointCloud>(stcg->simStates[0]->pointCloud);

	for (const ControlPoint& mp : stcg->outputPointCloud->controlPoints) {

		std::cout << "mp.F:\n" << glm::to_string(mp.F) << std::endl;
		std::cout << "mp.dLdF:\n" << glm::to_string(mp.dLdF) << std::endl;
	}

	//ResetControlPointCloudPointsToSaved(controlPointCloud);
	//std::cout.precision(prevPrecision);

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

