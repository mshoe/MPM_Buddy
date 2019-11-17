#include "MpmSpaceTimeControl.h"

mpm::control::ControlPointCloud::ControlPointCloud(std::shared_ptr<const PointCloud> pointCloud)
{
	std::cout << "Creating control point cloud from regular point cloud...\n";
	controlPoints.resize(pointCloud->points.size());
	for (size_t i = 0; i < pointCloud->points.size(); i++) {
		controlPoints[i] = ControlPoint(pointCloud->points[i]);
	}
	GenControlPointCloudSSBO();
}

mpm::control::ControlPointCloud::ControlPointCloud(std::shared_ptr<const ControlPointCloud> pointCloud)
{
	std::cout << "Creating copy point cloud from control point cloud...\n";
	controlPoints.resize(pointCloud->controlPoints.size());
	for (size_t i = 0; i < pointCloud->controlPoints.size(); i++) {
		controlPoints[i] = pointCloud->controlPoints[i];
	}
	GenControlPointCloudSSBO();
}

void mpm::control::ControlPointCloud::SetRegularPointCloud(std::shared_ptr<PointCloud> pointCloud)
{
	//controlPoints.resize(pointCloud->points.size());

	if (controlPoints.size() != pointCloud->points.size()) {
		std::cout << "Error, point clouds not same size.\n";
		return;
	}

	for (size_t i = 0; i < pointCloud->points.size(); i++) {
		controlPoints[i].SetRegularMaterialPoint(pointCloud->points[i]);
	}
}

void mpm::control::ControlPointCloud::GenControlPointCloudSSBO()
{
	GLuint pointCloudSSBO;
	glCreateBuffers(1, &pointCloudSSBO);
	ssbo = pointCloudSSBO;
	glNamedBufferStorage(
		ssbo,
		sizeof(ControlPoint) * controlPoints.size(),
		controlPoints.data(),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // add write bit for cpu mode
	);

	//MapToGPU();
}

void mpm::control::ControlPointCloud::MapToGPU()
{
	void* ptr = glMapNamedBuffer(ssbo, GL_WRITE_ONLY);
	ControlPoint* data = static_cast<ControlPoint*>(ptr);
	memcpy(data, controlPoints.data(), controlPoints.size() * sizeof(ControlPoint));
	glUnmapNamedBuffer(ssbo);
}

mpm::control::ControlGrid::ControlGrid(int _xSize, int _ySize)
{
	grid_size_x = _xSize;
	grid_size_y = _ySize;
	nodes = std::vector<std::vector<ControlGridNode>>(_xSize, std::vector<ControlGridNode>(_ySize, ControlGridNode()));

	for (int i = 0; i < _xSize; i++) {
		for (int j = 0; j < _ySize; j++) {
			nodes[i][j].x = vec2(real(i), real(j));
		}
	}
}

