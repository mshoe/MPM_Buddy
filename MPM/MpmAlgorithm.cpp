#include "MpmAlgorithmEngine.h"


void mpm::MpmAlgorithmEngine::MpmTimeStep(real dt)
{

	real curr_dt = 0.0;
	real rt_dt = 1.0 / 60.0;
	int timeStepCount = 0;

	if (!m_rt) {
		MpmTimeStepAlgoSelector(m_dt);
		m_time += m_dt;
		m_timeStep += 1;
	}
	else {
		while (curr_dt < rt_dt) {
			MpmTimeStepAlgoSelector(m_dt);
			curr_dt += m_dt;
			timeStepCount += 1;
		}
		m_time += curr_dt;
		m_timeStep += timeStepCount;
	}

	m_mpmEngine->MapCPUPointCloudsToGPU();
	m_mpmEngine->MapCPUGridToGPU();
}

void mpm::MpmAlgorithmEngine::MpmTimeStepAlgoSelector(real dt)
{
	switch (m_mpm_algo) {
	case MPM_ALGO::MLS:
		MpmTimeStep_MLS(m_dt);
		break;
	case MPM_ALGO::MUSL:
		MpmTimeStep_MUSL(m_dt);
		break;
	case MPM_ALGO::USF:
		MpmTimeStep_USF(m_dt);
		break;
	case MPM_ALGO::SE:
		MpmTimeStep_SE(m_dt);
		break;
	default:
		break;
	}
	return;
}

void mpm::MpmAlgorithmEngine::Update()
{
	if (!m_paused) {
		switch (m_algo_code) {
		case MPM_ALGORITHM_CODE::GLSL:
			if (!m_rt) {
				MpmTimeStep_GLSL(m_dt);

			}
			else {
				real curr_dt = 0.0;
				real rt_dt = 1.0 / 60.0;
				while (curr_dt < rt_dt) {
					MpmTimeStep_GLSL(m_dt);
					curr_dt += m_dt;
				}
			}
			break;
		case MPM_ALGORITHM_CODE::CPP:
			MpmTimeStep(m_dt);
			break;
		default:
			break;
		}
	}
}

void mpm::MpmAlgorithmEngine::CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud)
{
	// Calculate volumes for the point cloud (volumes stored in SSBO on GPU)
	
	//if (m_algo_code == MPM_ALGORITHM_CODE::GLSL) {
	//	CalculatePointCloudVolumes_GLSL(pointCloudID, pointCloud);
	//}

	//if (m_algo_code == MPM_ALGORITHM_CODE::CPP) {
		// for CPU mode calculate volumes
		CalculatePointCloudVolumesCPP(pointCloudID, pointCloud);
		//GetPointCloudVolumesFromGPUtoCPU(pointCloudID, pointCloud);
	//}

	//// Set initial volumes too (vol0)
	//for (int i = 0; i < pointCloud->points.size(); i++) {
	//	pointCloud->points[i].vol0 = pointCloud->points[i].vol;
	//}
}

void mpm::MpmAlgorithmEngine::CalculatePointCloudVolumesCPP(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud)
{

	// reset the grid masses
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {
			size_t index = i + GRID_SIZE_Y * j;
			m_mpmEngine->m_grid->nodes[index].m = 0.0;
		}
	}

	// particle to grid : masses
	for (MaterialPoint& mp : pointCloud->points) {

		nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

		while (!nodeGetter.Complete()) {
			GridNode& node = nodeGetter.NextNode();
			if (nodeGetter.IsNodeOK()) {

				vec2 xg = node.x;
				vec2 dpg = xg - mp.x;
				real dx = dpg.x; // sign matters for gradient
				real dy = dpg.y;
				real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);


				// P2G mass
				node.m += wpg * mp.m;
			}
		}
	}

	// grid to particle : densities
	for (MaterialPoint& mp : pointCloud->points) {

		nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

		double density_0 = 0.0;

		while (!nodeGetter.Complete()) {
			GridNode& node = nodeGetter.NextNode();
			if (nodeGetter.IsNodeOK()) {

				vec2 xg = node.x;
				vec2 dpg = xg - mp.x;
				real dx = dpg.x; // sign matters for gradient
				real dy = dpg.y;
				real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);


				// P2G mass
				density_0 += wpg * node.m; // wpg * m / h^2, but h is 1.0 in this program
			}
		}

		mp.vol0 = mp.m / density_0;
		mp.vol = mp.vol0;
	}
}

void mpm::MpmAlgorithmEngine::ChangeEnergyModel(ENERGY_MODEL comodel)
{
	// save the current material parameters

	if (comodel >= ENERGY_MODEL::Count) {
		return; // not sure if this error check makes sense
	}

	m_energyModels[size_t(m_comodel)] = m_mpParameters;
	m_comodel = comodel;
	m_mpParameters = m_energyModels[size_t(comodel)];
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP_Stress(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {
			UpdateStress(mp, pointCloudPair.second->comodel);
		}
	}
}