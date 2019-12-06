#include "MpmSpaceTimeControl.h"

mpm::control::ControlPointCloud::ControlPointCloud(std::shared_ptr<const PointCloud> pointCloud)
{
	std::cout << "Creating control point cloud from regular point cloud...\n";
	controlPoints.resize(pointCloud->points.size());
	for (size_t i = 0; i < pointCloud->points.size(); i++) {
		controlPoints[i] = ControlPoint(pointCloud->points[i]);
	}
	ComputeTotalMass();
}

mpm::control::ControlPointCloud::ControlPointCloud(std::shared_ptr<const ControlPointCloud> pointCloud)
{
	std::cout << "Creating copy point cloud from control point cloud...\n";
	controlPoints.resize(pointCloud->controlPoints.size());
	for (size_t i = 0; i < pointCloud->controlPoints.size(); i++) {
		controlPoints[i] = pointCloud->controlPoints[i];
	}
	totalMass = pointCloud->totalMass;
}

void mpm::control::ControlPointCloud::SetFromControlPointCloud(std::shared_ptr<const ControlPointCloud> pointCloud)
{
	if (controlPoints.size() != pointCloud->controlPoints.size()) {
		std::cout << "Error, point clouds not same size.\n";
		return;
	}

	for (size_t i = 0; i < pointCloud->controlPoints.size(); i++) {
		controlPoints[i] = pointCloud->controlPoints[i];
	}
	totalMass = pointCloud->totalMass;
}

void mpm::control::ControlPointCloud::SetFromPreviousTimeStepControlPointCloud(std::shared_ptr<const ControlPointCloud> pointCloud)
{
	if (controlPoints.size() != pointCloud->controlPoints.size()) {
		std::cout << "Error, point clouds not same size.\n";
		return;
	}

	for (size_t i = 0; i < pointCloud->controlPoints.size(); i++) {
		controlPoints[i].SetFromPreviousTimeStepControlPoint(pointCloud->controlPoints[i]);
	}
	totalMass = pointCloud->totalMass;
}

void mpm::control::ControlPointCloud::SetRegularPointCloud(std::shared_ptr<PointCloud> pointCloud)
{
	//controlPoints.resize(pointCloud->points.size());

	if (controlPoints.size() != pointCloud->points.size()) {
		std::cout << "Error, point clouds not same size.\n";
		return;
	}

	std::cout << "Setting point cloud from control point cloud..." << std::endl;

	for (size_t i = 0; i < pointCloud->points.size(); i++) {
		controlPoints[i].SetRegularMaterialPoint(pointCloud->points[i]);
	}
}

void mpm::control::ControlPointCloud::SetF(mat2 F)
{
	for (size_t i = 0; i < controlPoints.size(); i++) {
		controlPoints[i].F = F;
	}
}

void mpm::control::ControlPointCloud::SetPointCloudMassEqualToGiven(std::shared_ptr<const ControlPointCloud> pointCloud)
{
	real massPerMP = pointCloud->totalMass / real(controlPoints.size());

	std::streamsize prevPrec = std::cout.precision(16);
	std::cout << "New mass per MP = " << massPerMP << std::endl;
	std::cout.precision(prevPrec);

	for (ControlPoint& mp : controlPoints) {
		mp.m = massPerMP;
	}
	// could just as easily set total mass,
	// but do this just to see how much numerical error was introduced
	ComputeTotalMass();
}



mpm::control::ControlGrid::ControlGrid(int _xSize, int _ySize)
{
	InitGrid(_xSize, _ySize);
}

void mpm::control::ControlGrid::InitGrid(int _xSize, int _ySize)
{
	grid_size_x = _xSize;
	grid_size_y = _ySize;
	nodes = std::vector<ControlGridNode>(_xSize * _ySize, ControlGridNode());

	for (int i = 0; i < _xSize; i++) {
		for (int j = 0; j < _ySize; j++) {
			size_t grid_ind = i + j * size_t(grid_size_y);
			nodes[grid_ind].x = vec2(real(i), real(j));
		}
	}
}


mpm::control::TargetGrid::TargetGrid(int _xSize, int _ySize)
{
	InitGrid(_xSize, _ySize);
}


mpm::control::MPMSpaceTimeComputationGraph::MPMSpaceTimeComputationGraph()
{
	std::shared_ptr<ControlGrid> tempGrid = std::make_shared<ControlGrid>(GRID_SIZE_X, GRID_SIZE_Y);
	GenControlGridSSBO(tempGrid, targetGridSsbo);
	GenControlGridSSBO(tempGrid, gridSsbo);
}

void mpm::control::MPMSpaceTimeComputationGraph::InitSTCG()
{
	originalPointCloud = std::make_shared<ControlPointCloud>(controlPointCloud);

	simStates.clear();

	// First initialize the spacetime computation graph
	for (int i = 0; i < timeSteps; i++) {
		simStates.push_back(
			std::make_shared<MPMSpaceComputationGraph>(
				std::make_shared<ControlPointCloud>(originalPointCloud),
				std::make_shared<ControlGrid>(grid_size_x, grid_size_y))
		);
	}
}

void mpm::control::MPMSpaceTimeComputationGraph::SetGridSize(int _grid_size_x, int _grid_size_y)
{
	grid_size_x = _grid_size_x;
	grid_size_y = _grid_size_y;

	targetGrid = std::make_shared<TargetGrid>(grid_size_x, grid_size_y);
}

void mpm::control::MPMSpaceTimeComputationGraph::InitControlPointCloud(std::shared_ptr<PointCloud> pointCloud)
{
	controlPointCloud = std::make_shared<ControlPointCloud>(pointCloud);
	GenControlPointCloudSSBO(controlPointCloud, controlSsbo);
}

void mpm::control::MPMSpaceTimeComputationGraph::InitTargetPointCloud(std::shared_ptr<PointCloud> pointCloud)
{
	targetPointCloud = std::make_shared<control::ControlPointCloud>(pointCloud);
	targetPointCloud->color = glm::highp_fvec4(1.f, 1.f, 0.f, 1.f);
	GenControlPointCloudSSBO(targetPointCloud, targetSsbo);
}

void mpm::control::MPMSpaceTimeComputationGraph::InitTargetPointCloud(std::shared_ptr<ControlPointCloud> pointCloud)
{
	targetPointCloud = std::make_shared<control::ControlPointCloud>(pointCloud);
	targetPointCloud->color = glm::highp_fvec4(1.f, 1.f, 0.f, 1.f);
	GenControlPointCloudSSBO(targetPointCloud, targetSsbo);
}
