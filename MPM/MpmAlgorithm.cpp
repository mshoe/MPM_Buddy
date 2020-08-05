#include "MpmAlgorithmEngine.h"

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
			if (!m_rt) {
				//MpmTimeStep_CPP(m_dt);
				MpmTimeStep_MUSL(m_dt);
				m_mpmEngine->MapCPUPointCloudsToGPU();
				m_mpmEngine->MapCPUGridToGPU();
			}
			else {
				real curr_dt = 0.0;
				real rt_dt = 1.0 / 60.0;
				while (curr_dt < rt_dt) {
					//MpmTimeStep_CPP(m_dt);
					MpmTimeStep_MUSL(m_dt);
					curr_dt += m_dt;
				}
				m_mpmEngine->MapCPUPointCloudsToGPU();
				m_mpmEngine->MapCPUGridToGPU();
			}
			break;
		default:
			break;
		}
	}
}

void mpm::MpmAlgorithmEngine::CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud)
{
	// Calculate volumes for the point cloud (volumes stored in SSBO on GPU)
	CalculatePointCloudVolumes_GLSL(pointCloudID, pointCloud);

	if (m_algo_code == MPM_ALGORITHM_CODE::CPP) {
		// for CPU mode calculate volumes
		GetPointCloudVolumesFromGPUtoCPU(pointCloudID, pointCloud);
	}

	// Set initial volumes too (vol0)
	for (int i = 0; i < pointCloud->points.size(); i++) {
		pointCloud->points[i].vol0 = pointCloud->points[i].vol;
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