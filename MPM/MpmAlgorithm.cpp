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


	// For each point, calculate kinetic and strain energy
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {
			mp.se = mp.SE(pointCloudPair.second->comodel);
			mp.delta_SE = mp.se;
			mp.ke = mp.KE();
			mp.delta_KE = mp.ke;
		}
	}


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




	// Measure change in kinetic and strain energy on points
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {
			mp.se = mp.SE(pointCloudPair.second->comodel);
			mp.delta_SE = mp.se - mp.delta_SE;
			mp.ke = mp.KE();
			mp.delta_KE = mp.ke - mp.delta_KE;
		}
	}

	return;
}

void mpm::MpmAlgorithmEngine::Update()
{
	if (!m_paused) {
		MpmTimeStep(m_dt);
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
	for (size_t i = 0; i < m_mpmEngine->m_grid->grid_dim_x; i++) {
		for (size_t j = 0; j < m_mpmEngine->m_grid->grid_dim_y; j++) {
			GridNode& node = m_mpmEngine->m_grid->GetNode(i, j);
			node.m = 0.0;
			//std::cout << node.x.x << ", " << node.x.y << std::endl;
		}
	}

	// particle to grid : masses
	for (MaterialPoint& mp : pointCloud->points) {

		nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

		//std::cout << nodeGetter.botLeftNodeIndex.x << ", " << nodeGetter.botLeftNodeIndex.y << std::endl;

		while (!nodeGetter.Complete()) {
			GridNode& node = nodeGetter.NextNode();
			if (nodeGetter.IsNodeOK()) {

				vec2 xg = node.x;
				vec2 dpg = xg - mp.x;
				real dx = dpg.x; // sign matters for gradient
				real dy = dpg.y;
				real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);

				

				//std::cout << xg.x << ", " << xg.y << std::endl;
				//std::cout << nodeGetter.botLeftNodeIndex.x + nodeGetter.i << ", " << nodeGetter.botLeftNodeIndex.y + nodeGetter.j << std::endl << std::endl;
				
				//std::cout << mp.x.x << ", " << mp.x.y << std::endl << std::endl;
				//std::cout << dpg.x << ", " << dpg.y << std::endl;

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

void mpm::MpmAlgorithmEngine::MpmReset()
{
	m_mpmEngine->m_pointCloudMap.clear();
	m_mpmEngine->m_circleCount = 0;
	m_mpmEngine->m_rectCount = 0;
	m_mpmEngine->m_isoTriCount = 0;
	m_mpmEngine->m_polygonCount = 0;
	m_mpmEngine->m_pwLineCount = 0;
	m_timeStep = 0;
	m_time = 0.0;
	/*glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_mpmEngine->gridSSBO);
	m_gReset->Use();
	glDispatchCompute(m_mpmEngine->m_chunks_x, m_mpmEngine->m_chunks_y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);*/
}
