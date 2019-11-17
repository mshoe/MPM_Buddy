#include "MpmControlEngine.h"


//void mpm::MpmControlEngine::SaveControlPointCloudOriginalPoints(std::shared_ptr<PointCloud> pointCloud)
//{
//	m_controlPointCloudOriginalPoints = pointCloud->points;
//	std::cout << "Saved control points" << std::endl;
//}
//
//void mpm::MpmControlEngine::ResetControlPointCloudPointsToSaved(std::shared_ptr<PointCloud> pointCloud)
//{
//	if (pointCloud->points.size() != m_controlPointCloudOriginalPoints.size()) {
//		std::cout << "Saved point cloud does not have the same number of points as selected point cloud" << std::endl;
//		return;
//	}
//	pointCloud->points = m_controlPointCloudOriginalPoints;
//	std::cout << "Reset selected point cloud to saved points" << std::endl;
//	m_mpmEngine->MapCPUPointCloudToGPU(pointCloud);
//}


