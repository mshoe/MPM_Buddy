#include "MpmEngine.h"

void mpm::MpmEngine::CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud)
{
	// Calculate volumes for the point cloud (volumes stored in SSBO on GPU)
	CalculatePointCloudVolumes_GLSL(pointCloudID, pointCloud);

	if (m_algo_code == MPM_ALGORITHM_CODE::CPP) {
		// for CPU mode calculate volumes
		GetPointCloudVolumesFromGPUtoCPU(pointCloudID, pointCloud);
	}
}