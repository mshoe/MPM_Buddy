#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"

void mpm::MpmEngine::SetDeformationGradients(std::string pointCloudID, mat2 Fe, mat2 Fp, bool setSelected)
{
	if (m_pointCloudMap.count(pointCloudID)) {
		std::shared_ptr<PointCloud> pointCloud = m_pointCloudMap[pointCloudID];
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
		m_pSetDeformationGradients->Use();
		m_pSetDeformationGradients->SetMat("setFe", Fe);
		m_pSetDeformationGradients->SetMat("setFp", Fp);
		m_pSetDeformationGradients->SetBool("setSelected", setSelected);
		int p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
}

void mpm::MpmEngine::MultiplyDeformationGradients(std::string pointCloudID, mat2 multFe, mat2 multFp, bool multSelected)
{
	if (m_pointCloudMap.count(pointCloudID)) {
		std::shared_ptr<PointCloud> pointCloud = m_pointCloudMap[pointCloudID];
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
		m_pMultDeformationGradients->Use();
		m_pMultDeformationGradients->SetMat("multFe", multFe);
		m_pMultDeformationGradients->SetMat("multFp", multFp);
		m_pMultDeformationGradients->SetBool("multSelected", multSelected);
		int p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
}


void mpm::MpmEngine::SelectPointsInPolygon(std::string pointCloudID)
{
	if (m_pointCloudMap.count(pointCloudID)) {
		std::shared_ptr<PointCloud> pointCloud = m_pointCloudMap[pointCloudID];
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
		
		GLuint polygonSSBO;
		glCreateBuffers(1, &polygonSSBO);

		glNamedBufferStorage(
			polygonSSBO,
			sizeof(vec2) * m_polygon->vertices.size(),
			&(m_polygon->vertices.front()),
			GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // adding write bit for debug purposes
		);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, polygonSSBO);
		
		m_pLassoTool->Use();
		int p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, 0);
		glDeleteBuffers(1, &polygonSSBO);
	}
}

void mpm::MpmEngine::ClearPointSelection(std::string pointCloudID)
{
	if (m_pointCloudMap.count(pointCloudID)) {
		std::shared_ptr<PointCloud> pointCloud = m_pointCloudMap[pointCloudID];
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);


		m_pClearPointSelection->Use();
		int p_workgroups = int(glm::ceil(real(pointCloud->N) / real(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
}


void mpm::MpmEngine::SelectNodesInShape(sdf::Shape& shape, const int gridDimX, const int gridDimY, const real inner_rounding, const real outer_rounding, sdf::SDF_OPTION sdfOption, bool inverted)
{
	//	std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();

	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	GridNode* data = static_cast<GridNode*>(ptr);


	int count = 0;
	// gen points from sdf
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			GridNode currNode = data[i * GRID_SIZE_Y + j];

			glm::vec2 p;
			p.x = real(i);
			p.y = real(j);// (real(i), real(j));

			real sd = 0.0;
			switch (sdfOption) {
			case sdf::SDF_OPTION::NORMAL:
				sd = shape.Sdf(p);
				break;
			case sdf::SDF_OPTION::ROUNDED:
				sd = shape.SdfRounded(p, outer_rounding);
				break;
			case sdf::SDF_OPTION::HOLLOW:
				sd = shape.SdfHollow(p, inner_rounding, outer_rounding);
				break;
			default:
				break;
			}

			if (inverted)
				sd *= -1.0;

			if (sd < 0.0) {
				currNode.selected = true;
				//currNode.force = vec2(100.0, 100.0);
				count++;
			}
			else {
				//currNode.selected = false;
			}

			data[i * GRID_SIZE_Y + j] = currNode;
		}
	}
	std::cout << "Selected " << count << " nodes.\n";
	glUnmapNamedBuffer(gridSSBO);

	/*ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	data = static_cast<GridNode*>(ptr);
	int count2 = 0;
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			GridNode currNode = data[i * GRID_SIZE_Y + j];

			if (currNode.selected)
				count2++;
		}
	}

	std::cout << "count2 = " << count2 << std::endl;
	glUnmapNamedBuffer(gridSSBO);*/

}

void mpm::MpmEngine::ClearNodesSelected(const int gridDimX, const int gridDimY)
{
	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	GridNode* data = static_cast<GridNode*>(ptr);


	int count = 0;
	// gen points from sdf
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			GridNode currNode = data[i * GRID_SIZE_Y + j];

			if (currNode.selected) {
				currNode.selected = false;
				count++;
				data[i * GRID_SIZE_Y + j] = currNode;
			}

		}
	}
	std::cout << "Cleared selection of " << count << " nodes.\n";
	glUnmapNamedBuffer(gridSSBO);
}

void mpm::MpmEngine::CalculateNodalAccelerations(const int gridDimX, const int gridDimY, real accStr)
{
	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	GridNode* data = static_cast<GridNode*>(ptr);

	std::vector<std::vector<vec2>> nodal_accs(gridDimX, std::vector<vec2>(gridDimY, vec2(0.0)));

	// temp buffer for node selected var
	int count = 0;
	std::vector<std::vector<bool>> nodes_selected(gridDimX, std::vector<bool>(gridDimY, false));
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			if (data[i * GRID_SIZE_Y + j].selected) {
				nodes_selected[i][j] = data[i * GRID_SIZE_Y + j].selected;
				count++;
			}
		}
	}

	if (count == 0) {
		glUnmapNamedBuffer(gridSSBO);
		std::cout << "Can't compute accelerations since no nodes selected.\n";
		return;
	}

	int countIter = 0;
	bool finished = false;
	// gen points from sdf
	while (!finished) {
		finished = true;
		for (int i = 0; i < gridDimX; i++) {
			for (int j = 0; j < gridDimY; j++) {

				GridNode currNode = data[i * GRID_SIZE_Y + j];

				if (currNode.selected) {
					continue; // don't calculate nodal acceleration for marked nodes
				}

				finished = false;

				int left = (i - 1) * GRID_SIZE_Y + j;
				int right = (i + 1) * GRID_SIZE_Y + j;
				int up = i * GRID_SIZE_Y + j + 1;
				int down = i * GRID_SIZE_Y + j - 1;


				// if left exists
				if (i > 0 && data[left].selected) {
					nodal_accs[i][j].x -= accStr;
					nodes_selected[i][j] = true;
				}
				// if right exists
				if (i < gridDimX - 1 && data[right].selected) {
					nodal_accs[i][j].x += accStr;
					nodes_selected[i][j] = true;
				}
				// if up exists
				if (j < gridDimY - 1 && data[up].selected) {
					nodal_accs[i][j].y += accStr;
					nodes_selected[i][j] = true;
				}
				// if down exists 
				if (j > 0 && data[down].selected) {
					nodal_accs[i][j].y -= accStr;
					nodes_selected[i][j] = true;
				}

				//currNode.nodalAcceleration = acc;
				//currNode.selected = true;


				data[i * GRID_SIZE_Y + j] = currNode;
			}
		}

		// Write buffer to original data
		for (int i = 0; i < gridDimX; i++) {
			for (int j = 0; j < gridDimY; j++) {
				GridNode currNode = data[i * GRID_SIZE_Y + j];
				currNode.selected = nodes_selected[i][j];
				currNode.nodalAcceleration = nodal_accs[i][j];
				data[i * GRID_SIZE_Y + j] = currNode;
			}
		}

		countIter++;
	}
	std::cout << "Computed nodal accelerations in " << countIter << " iterations.\n";
	glUnmapNamedBuffer(gridSSBO);
}

void mpm::MpmEngine::ClearNodalAcclerations(const int gridDimX, const int gridDimY)
{
	void* ptr = glMapNamedBuffer(gridSSBO, GL_READ_WRITE);
	GridNode* data = static_cast<GridNode*>(ptr);


	int count = gridDimX * gridDimY;
	// gen points from sdf
	for (int i = 0; i < gridDimX; i++) {
		for (int j = 0; j < gridDimY; j++) {

			GridNode currNode = data[i * GRID_SIZE_Y + j];


			currNode.nodalAcceleration = vec2(0.0);
			currNode.selected = false;

			data[i * GRID_SIZE_Y + j] = currNode;
		}
	}
	std::cout << "Cleared " << count << " nodal accelerations.\n";
	glUnmapNamedBuffer(gridSSBO);
}