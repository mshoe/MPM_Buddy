#include "MpmSpaceTimeControl.h"

mpm::control::ControlPoint::ControlPoint(const MaterialPoint& mp)
{
	F = mp.Fe;
	dLdF = mat2(0.0);
	P = mp.P;
	dLdP = mat2(0.0);
	C = mp.B;
	dLdC = mat2(0.0);
	x = mp.x;
	dLdx = vec2(0.0);
	v = mp.v;
	dLdv = vec2(0.0);
	m = mp.m;
	vol = mp.vol;
	lam = mp.lam;
	mew = mp.mew;
	rgba = mp.rgba;
}

void mpm::control::ControlPoint::SetRegularMaterialPoint(MaterialPoint& mp)
{
	mp.x = x;
	mp.v = v;
	mp.B = C;
	mp.Fe = F + dFc;
	mp.P = P;
	mp.m = m;
	mp.vol = vol;
	mp.lam = lam;
	mp.mew = mew;
	mp.rgba = rgba;
}

void mpm::control::ControlPoint::SetFromPreviousTimeStepControlPoint(const ControlPoint& mp)
{
	F = mp.F;
	P = mp.P;
	C = mp.C;
	x = mp.x;
	v = mp.v;
	m = mp.m;
	vol = mp.vol;
	lam = mp.lam;
	mew = mp.mew;
	rgba = mp.rgba;
}

void mpm::control::ControlPoint::ResetGradients()
{
	dLdF = mat2(0.0);
	dLdP = mat2(0.0);
	dLdC = mat2(0.0);
	dLdx = vec2(0.0);
	dLdv = vec2(0.0);
}

void mpm::control::ControlPoint::ImGuiDisplay()
{
	glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
	glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);

	ImGui::DisplayNamedGlmVecMixColor("x", x, min_color, max_color);
	ImGui::DisplayNamedGlmVecMixColor("dLdx", dLdx, min_color, max_color);
	ImGui::DisplayNamedGlmVecMixColor("v", v, min_color, max_color);
	ImGui::DisplayNamedGlmVecMixColor("dLdc", dLdv, min_color, max_color);
	ImGui::DisplayNamedGlmRealColor("m", m, max_color);
	ImGui::DisplayNamedGlmRealColor("dLdm", dLdm, max_color);
	ImGui::DisplayNamedGlmRealColor("vol", vol, max_color);
	ImGui::DisplayNamedGlmRealColor("dLdvol", dLdvol, max_color);
	ImGui::DisplayNamedGlmRealColor("lam", lam, max_color);
	ImGui::DisplayNamedGlmRealColor("dlamc", dlamc, max_color);
	ImGui::DisplayNamedGlmRealColor("dLdlam", dLdlam, max_color);
	ImGui::DisplayNamedGlmRealColor("mew", mew, max_color);
	ImGui::DisplayNamedGlmRealColor("dmewc", dmewc, max_color);
	ImGui::DisplayNamedGlmRealColor("dLdmew", dLdmew, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("F", F, min_color, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("dFc", dFc, min_color, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("dLdF", dLdF, min_color, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("P", P, min_color, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("dLdP", dLdP, min_color, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("C", C, min_color, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("dLdC", dLdC, min_color, max_color);
}

void mpm::control::ControlGridNode::Reset_vpm()
{
	v = vec2(0.0);
	p = vec2(0.0);
	m = 0.0;
}

void mpm::control::ControlGridNode::Reset_rgba()
{
	rgba = vec4(0.0, 0.0, 0.0, 1.0);
}

void mpm::control::ControlGridNode::ResetGradients()
{
	dLdv = vec2(0.0);
	dLdp = vec2(0.0);
	dLdm = 0.0;
}

void mpm::control::ControlGridNode::ImGuiDisplay()
{
	glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
	glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);
	ImGui::DisplayNamedGlmRealColor("m", m, max_color);
	ImGui::DisplayNamedGlmRealColor("dLdm", dLdm, max_color);

	ImGui::DisplayNamedGlmVecMixColor("x", x, min_color, max_color);

	ImGui::DisplayNamedGlmVecMixColor("p", p, min_color, max_color);
	ImGui::DisplayNamedGlmVecMixColor("dLdp", dLdp, min_color, max_color);

	ImGui::DisplayNamedGlmVecMixColor("v", v, min_color, max_color);
	ImGui::DisplayNamedGlmVecMixColor("dLdv", dLdv, min_color, max_color);

}



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

//std::shared_ptr<mpm::PointCloud> mpm::control::ControlPointCloud::GenRegularPointCloud()
//{
//	std::shared_ptr<PointCloud> ret = std::make_shared<PointCloud>();
//	SetRegularPointCloud(ret);
//	ret->GenPointCloudSSBO();
//	return ret;
//}

void mpm::control::ControlPointCloud::SetF(mat2 F)
{
	for (size_t i = 0; i < controlPoints.size(); i++) {
		controlPoints[i].F = F;
	}
}

void mpm::control::ControlPointCloud::ResetGradients()
{
	for (size_t p = 0; p < controlPoints.size(); p++) {
		controlPoints[p].ResetGradients();
	}
}

void mpm::control::ControlPointCloud::ResetdFc()
{
	for (size_t p = 0; p < controlPoints.size(); p++) {
		controlPoints[p].dFc = mat2(0.0);
	}
}

bool mpm::control::ControlPointCloud::CheckFConvergence(real tol, bool debugOutput)
{
	real norm = 0.0;
	for (ControlPoint& mp : controlPoints) {
		norm += MatrixNormSqrd(mp.dLdF);
	}
	norm = sqrt(norm);
	if (debugOutput)
		std::cout << "norm is: " << norm << std::endl;
	if (norm < tol) {
		std::cout << "norm under tolerance..." << std::endl;
		return true;
	}
	else if (isnan(norm)) {
		std::cout << "gradient is nan..." << std::endl;
		return true;
	}
	return false;
}

void mpm::control::ControlPointCloud::DescendFGradients(real alpha)
{
	for (ControlPoint& mp : controlPoints) {
		// view this dLdF matrix as a vector
		// descend in the direction of the vector
		mp.dFc -= alpha * mp.dLdF;
	}
}

void mpm::control::ControlPointCloud::DescendMaterialGradients(real alpha)
{
	// First properly normalize the dL / dF's
	real norm = 0.0;
	for (ControlPoint& mp : controlPoints) {
		norm += mp.dLdlam * mp.dLdlam;
		norm += mp.dLdmew * mp.dLdmew;
	}
	norm = sqrt(norm);

	if (norm == 0.0) {
		std::cout << "rare error: norm = 0" << std::endl;
		return;
	}

	for (ControlPoint& mp : controlPoints) {
		mp.dlamc -= alpha * mp.dLdlam / norm;
		mp.dmewc -= alpha * mp.dLdmew / norm;
	}
}

void mpm::control::ControlPointCloud::ComputeTotalMass()
{
	std::cout << "computing total mass..." << std::endl;
	real mass = 0.0;
	for (const ControlPoint& mp : controlPoints) {
		mass += mp.m;
	}
	totalMass = mass;
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

void mpm::control::ControlPointCloud::SetFsToIdentity()
{
	for (ControlPoint& mp : controlPoints) {
		mp.F = mat2(1.0);
		//mp.dFc = mat2(0.0);
	}
}

bool mpm::control::ControlPointCloud::Check_dLdF_Nan()
{
	for (ControlPoint& mp : controlPoints) {
		if (glm::isnan(mp.dLdF[0][0]) ||
			glm::isnan(mp.dLdF[0][1]) ||
			glm::isnan(mp.dLdF[1][0]) ||
			glm::isnan(mp.dLdF[1][1])) {
			return true;
		}
	}
	return false;
}



mpm::control::ControlGrid::ControlGrid(int _xSize, int _ySize)
{
	InitGrid(_xSize, _ySize);
}

mpm::control::ControlGrid::~ControlGrid()
{
	nodes.clear(); // vector clears recursively
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

mpm::control::ControlGridNode& mpm::control::ControlGrid::Node(size_t i, size_t j)
{
	return nodes[i + grid_size_y * j];
}

const mpm::control::ControlGridNode& mpm::control::ControlGrid::ConstNode(size_t i, size_t j) const
{
	return nodes[i + grid_size_y * j];
}

void mpm::control::ControlGrid::ResetGradients()
{
	for (size_t i = 0; i < size_t(grid_size_x); i++) {
		for (size_t j = 0; j < size_t(grid_size_y); j++) {
			size_t grid_ind = i + j * size_t(grid_size_y);
			nodes[grid_ind].ResetGradients();
		}
	}
}

real mpm::control::ControlGrid::GetTotalMass()
{
	real totalMass = 0.0;
	for (size_t i = 0; i < size_t(grid_size_x); i++) {
		for (size_t j = 0; j < size_t(grid_size_y); j++) {
			size_t grid_ind = i + j * size_t(grid_size_y);
			totalMass += nodes[grid_ind].m;
		}
	}
	return totalMass;
}


mpm::control::TargetGrid::TargetGrid(int _xSize, int _ySize)
{
	InitGrid(_xSize, _ySize);
}

mpm::control::TargetGrid::~TargetGrid()
{
	penaltyWeights.clear();
	nodes.clear();
}

void mpm::control::TargetGrid::InitializePenaltyWeights(real penalty)
{
	penaltyWeights.resize(grid_size_x, std::vector<real>(grid_size_y, 1.0));
	for (int i = 0; i < grid_size_x; i++) {
		for (int j = 0; j < grid_size_y; j++) {
			if (Node(i, j).m == 0.0) {
				penaltyWeights[i][j] = penalty;
			}
		}
	}
}


mpm::control::MPMSpaceTimeComputationGraph::MPMSpaceTimeComputationGraph()
{
	std::shared_ptr<ControlGrid> tempGrid = std::make_shared<ControlGrid>(GRID_SIZE_X, GRID_SIZE_Y);
	GenControlGridSSBO(tempGrid, targetGridSsbo);
	GenControlGridSSBO(tempGrid, gridSsbo);
}

mpm::control::MPMSpaceTimeComputationGraph::~MPMSpaceTimeComputationGraph()
{
	glDeleteBuffers(1, &controlSsbo);
	glDeleteBuffers(1, &targetSsbo);
	glDeleteBuffers(1, &gridSsbo);
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

void mpm::control::MPMSpaceTimeComputationGraph::InitControlPointCloud(std::shared_ptr<ControlPointCloud> pointCloud)
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

void mpm::control::MPMSpaceTimeComputationGraph::SetTargetPointCloudMassToControl()
{
	if (targetPointCloud != nullptr && controlPointCloud != nullptr) {
		targetPointCloud->SetPointCloudMassEqualToGiven(controlPointCloud);
	}
	else {
		std::cout << "error" << std::endl;
	}
}
