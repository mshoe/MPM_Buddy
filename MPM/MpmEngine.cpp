#include "MpmEngine.h"
#include "imgui/imgui.h"

float BSpline(float x) {
	return (x < 0.5f) ? glm::step(0.0f, x)*(0.75f - x * x) :
		glm::step(x, 1.5f)*0.5f*(1.5f - abs(x))*(1.5f - abs(x));
}

float BSplineSlope(float x) {
	return (x < 0.5f) ? glm::step(0.0f, x)*(-2.f * x) :
		glm::step(x, 1.5f)*(1.5f - abs(x))*x / abs(x);
}



bool mpm::MpmEngine::InitComputeShaderPipeline()
{
	using namespace std::chrono;

	glEnable(GL_PROGRAM_POINT_SIZE);

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	int work_group_size;
	glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &work_group_size);
	std::cout << "max work group size: " << work_group_size << std::endl;

	// first compile shaders
	m_gReset = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\gResetNodes.comp"}, "shaders\\compute\\mpm_header.comp");
	m_p2gScatter = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\p2gScatterParticleAndUpdateNodes.comp"}, "shaders\\compute\\mpm_header.comp");
	m_p2gGather = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\p2gGatherParticlesAndUpdateNode.comp"}, "shaders\\compute\\mpm_header.comp");
	m_gUpdate = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\gUpdateNodes.comp"}, "shaders\\compute\\mpm_header.comp");

	m_g2pGather = std::make_unique<ComputeShader>(std::vector<std::string>{"shaders\\compute\\g2pGatherNodesAndUpdateParticle.comp"}, "shaders\\compute\\mpm_header.comp");
	m_p2gCalcVolumes = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\p2gCalculateVolumes.comp"}, "shaders\\compute\\mpm_header.comp");
	m_g2pCalcVolumes = std::make_unique<ComputeShader>(std::vector<std::string> {"shaders\\compute\\g2pCalculateVolumes.comp"}, "shaders\\compute\\mpm_header.comp");

	glCreateVertexArrays(1, &VisualizeVAO);

	m_pPointCloudShader = std::make_unique<StandardShader>(std::vector<std::string>{"shaders\\graphics\\pointCloud.vs"}, std::vector<std::string>{"shaders\\graphics\\pointCloud.fs"}, "shaders\\compute\\mpm_header.comp");
	m_mouseShader = std::make_unique<StandardShader>(std::vector<std::string>{"shaders\\graphics\\mouseShader.vs"}, std::vector<std::string>{"shaders\\graphics\\mouseShader.fs"}, "shaders\\compute\\mpm_header.comp");


	// Initialize the grid SSBO on the GPU
	m_grid = Grid(GRID_SIZE_X, GRID_SIZE_Y);

	glCreateBuffers(1, &gridSSBO);

	glNamedBufferStorage(
		gridSSBO,
		sizeof(GridNode) * GRID_SIZE_X * GRID_SIZE_Y,
		&(m_grid.nodes[0].m),
		GL_MAP_READ_BIT
	);


	CreateDemo();

	return true;
}

bool mpm::MpmEngine::CleanupComputeShaderPipeline()
{
	m_pointCloudMap.clear();
	glDeleteBuffers(1, &gridSSBO);
	glDeleteVertexArrays(1, &VisualizeVAO);

	return false;
}

void mpm::MpmEngine::MpmTimeStep(float dt)
{
#ifdef MPM_DEBUG
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();
#endif
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gReset->Use();
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		/*m_p2gGather->Use();
		m_p2gGather->SetFloat("dt", dt);
		glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);*/
		m_p2gScatter->Use();
		m_p2gScatter->SetFloat("dt", dt);
		int g2p_workgroups = int(glm::ceil(float(pointCloudPair.second->N) / float(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(g2p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	m_gUpdate->Use();
	m_gUpdate->SetFloat("dt", dt);
	m_gUpdate->SetVec("globalForce", m_globalForce);
	m_gUpdate->SetVec("mousePos", m_mousePos);
	if (m_rightButtonDown)
		m_gUpdate->SetFloat("mousePower", m_mousePower);
	else
		m_gUpdate->SetFloat("mousePower", 0.f);
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
		m_g2pGather->Use();
		m_g2pGather->SetFloat("dt", dt);
		m_g2pGather->SetFloat("lam", pointCloudPair.second->lam);
		m_g2pGather->SetFloat("mew", pointCloudPair.second->mew);
		int g2p_workgroups = int(glm::ceil(float(pointCloudPair.second->N) / float(G2P_WORKGROUP_SIZE)));
		glDispatchCompute(g2p_workgroups, 1, 1);
		glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	}
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);
	m_timeStep++;
	m_time += dt;
#ifdef MPM_DEBUG
	t2 = high_resolution_clock::now();
	std::cout << "Finished calculating timestep in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
#endif
}

void mpm::MpmEngine::Render()
{
	if (!m_paused) {
		MpmTimeStep(m_dt);
	}
	m_pPointCloudShader->Use();
	glBindVertexArray(VisualizeVAO);
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		m_pPointCloudShader->SetVec("pointCloudColor", pointCloudPair.second->color);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glDrawArrays(GL_POINTS, 0, (GLsizei)pointCloudPair.second->N);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderGUI()
{
	if (m_renderGUI) {
		ImGui::Begin("MPM Grid Data", &m_renderGUI);

		
		if (ImGui::Button("Set global force")) {
			m_globalForce.x = m_globalForceArray[0];
			m_globalForce.y = m_globalForceArray[1];
		}
		ImGui::InputFloat2("", m_globalForceArray, "%.3f");

		ImGui::InputFloat("Mouse power", &m_mousePower);

		ImGui::Text(std::to_string(m_time).c_str());
		ImGui::Text(std::to_string(m_timeStep).c_str());
		ImGui::InputFloat("dt", &m_dt, 0.001f, 1.f/60.f, "%.6f");

		/*ImGui::InputFloat2("circle init x", m_circle_x, "%.3f");
		ImGui::InputFloat2("circle init v", m_circle_v, "%.3f");
		ImGui::InputFloat2("donut init x", m_donut_x, "%.3f");
		ImGui::InputFloat2("donut init v", m_donut_v, "%.3f");*/
		

		if (ImGui::Button("Create Solid Circle") && m_paused) {
			m_createCircleState = true;
		}

		
		if (ImGui::Button("Get node data") && m_paused) {
			UpdateNodeData();
		}
		ImGui::InputInt2("Grid Node:", m_node);
		ImGui::Text(m_nodeText.c_str());
		
		if (ImGui::Button("Pause")) {
			m_paused = !m_paused;
		}
		if (ImGui::Button("Advance") && m_paused) {
			MpmTimeStep(m_dt);
			UpdateNodeData();
		}


		if (ImGui::Button("Restart")) {
			CreateDemo();
			m_timeStep = 0;
			m_time = 0.f;
		}

		ImGui::End();

		/*ImGui::Begin("Material Point View");
		if (ImGui::Button("View Particles") && m_paused) {
			void *ptr = glMapNamedBuffer(pointCloudSSBO[m_pointCloudSelect], GL_READ_ONLY);
			MaterialPoint *data = static_cast<MaterialPoint*>(ptr);
			std::ostringstream pointsViewStr;
			for (size_t i = 0; i < m_pointClouds[m_pointCloudSelect].N; ++i) {
				pointsViewStr << "Material Point " << i << ":" << std::endl;
				pointsViewStr << data[i] << std::endl;
			}
			m_pointsViewStr = pointsViewStr.str();
			glUnmapNamedBuffer(pointCloudSSBO[m_pointCloudSelect]);
		}
		ImGui::Text(m_pointsViewStr.c_str());

		ImGui::End();*/
	}
}

void mpm::MpmEngine::HandleInput()
{

	if (m_paused && m_createCircleState && m_leftButtonDown)
	{
		m_createCircleState = false;

		float youngMod = 400.f;
		float poisson = 0.3f;

		using namespace std::chrono;

		time_point<high_resolution_clock> t1;
		time_point<high_resolution_clock> t2;

		std::cout << "Mouse position is at (" << m_mousePos.x << ", " << m_mousePos.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_circleCount++;
		sdf::sdFunc dCircle(sdf::DemoCircle);
		Shape shape(glm::vec2(m_mousePos.x * GRID_SIZE_X, m_mousePos.y * GRID_SIZE_Y), std::vector<float>{5.f});
		std::string circleID = "circle" + std::to_string(m_circleCount);
		GenPointCloud(circleID, shape, dCircle, GRID_SIZE_X, GRID_SIZE_Y, 0.25f, 0.16f, youngMod, poisson, glm::vec2(-3.f, 0.f), glm::vec3(1.f, 0.f, 0.f));

		t2 = high_resolution_clock::now();

		

		std::cout << "Finished generating " << m_pointCloudMap[circleID]->N << " points for '" << circleID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}
}


void mpm::MpmEngine::UpdateNodeData()
{
	if (0 <= m_node[0] && m_node[0] < GRID_SIZE_X && 0 <= m_node[1] && m_node[1] < GRID_SIZE_Y) {
		void *ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);
		GridNode *data = static_cast<GridNode*>(ptr);
		GridNode gn = data[m_node[0] * GRID_SIZE_X + m_node[1]];
		std::ostringstream nodeText;
		nodeText << gn << std::endl;
		m_nodeText = nodeText.str();
		glUnmapNamedBuffer(gridSSBO);
	}
}


std::shared_ptr<PointCloud> mpm::MpmEngine::GenPointCloud(const std::string pointCloudID, Shape& shape, sdf::sdFunc _sdf,
	const float gridDimX, const float gridDimY, const float particleSpacing, 
	const float density, const float youngMod, const float poisson,
	glm::vec2 initialVelocity, glm::vec3 color)
{
	std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();

	pointCloud->color = color;

	pointCloud->mew = youngMod / (2.f + 2.f*poisson);
	pointCloud->lam = youngMod * poisson / ((1.f + poisson) * (1.f - 2.f * poisson));

	float mass = particleSpacing * particleSpacing * density;
	
	// gen points from sdf
	for (float x = 0.f; x < gridDimX; x += particleSpacing) {
		for (float y = 0.f; y < gridDimY; y += particleSpacing) {
			
			glm::vec2 pos(x, y);
			float sd = _sdf(shape, pos);
			if (sd < 0.f) {
				MaterialPoint mp;
				mp.x = pos;
				mp.v = initialVelocity;
				mp.m = mass;
				// calculate mp.vol in a compute shader (not here)
				mp.B = glm::mat2(0.f);
				mp.F = glm::mat2(1.f);
				mp.P = glm::mat2(0.f); // initial Piola stress tensor is 0
				
				
				pointCloud->points.push_back(mp);
			}
		}
	}
	pointCloud->N = pointCloud->points.size();

	// Create the SSBO for the point cloud, so it is stored on the GPU
	GLuint pointCloudSSBO;
	glCreateBuffers(1, &pointCloudSSBO);
	pointCloud->ssbo = pointCloudSSBO;
	glNamedBufferStorage(
		pointCloud->ssbo,
		sizeof(MaterialPoint) * pointCloud->points.size(),
		&(pointCloud->points.front().x.x),
		GL_MAP_READ_BIT
	);

	// Calculate volumes for the point cloud (volumes stored in SSBO on GPU)
	CalculatePointCloudVolumes(pointCloudID, pointCloud);


	m_pointCloudMap[pointCloudID] = pointCloud;

	return pointCloud;
}

void mpm::MpmEngine::CreateDemo()
{
	using namespace std::chrono;

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	t1 = high_resolution_clock::now();
	m_pointCloudMap.clear();
	t2 = high_resolution_clock::now();
	std::cout << "Clearing point clouds took" << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";

	float youngMod = 400.f;
	float poisson = 0.3f;

	// 1. Create point clouds
	std::cout << "Generating point cloud...\n";

	m_circleCount++;

	t1 = high_resolution_clock::now();

	sdf::sdFunc dCircle(sdf::DemoCircle);
	Shape shape(glm::vec2(35.f, 10.f), std::vector<float>{5.f});
	GenPointCloud("circle1", shape, dCircle, GRID_SIZE_X, GRID_SIZE_Y, 0.25f, 0.16f, youngMod, poisson, glm::vec2(-3.f, 0.f), glm::vec3(1.f, 0.f, 0.f));
	
	t2 = high_resolution_clock::now();

	std::cout << "Finished generating " << m_pointCloudMap["circle1"]->N << " points for 'circle1' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	
	

	//t1 = high_resolution_clock::now();
	//sdf::sdFunc dDonut(sdf::DemoDonut);
	//Shape shape2(glm::vec2(15.f, 10.f), std::vector<float>{1.5f, 6.f});
	///*PointCloud sh2 = GenPointCloud(shape2, dDonut, GRID_SIZE_X, GRID_SIZE_Y, 0.25f, 0.16f, 100.f, 0.2f, glm::vec2(m_donut_v[0], m_donut_v[1]));
	//m_pointClouds[0].points.insert(m_pointClouds[0].points.end(), sh2.points.begin(), sh2.points.end());
	//m_pointClouds[0].N += sh2.N;*/
	//m_pointClouds.push_back(GenPointCloud(shape2, dDonut, GRID_SIZE_X, GRID_SIZE_Y, 0.25f, 0.16f, youngMod, poisson, glm::vec2(3.f, 0.f)));
	//m_pointClouds[index].color = glm::vec3(0.f, 1.f, 0.f);
	//t2 = high_resolution_clock::now();
	//std::cout << "Finished generating " << m_pointClouds[index].N << " points for DemoDonut point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	//index++;

	//sdf::sdFunc dDonut2(sdf::DemoDonut);
	//Shape shape3(glm::vec2(80.f, 80.f), std::vector<float>{1.0f, 5.f});
	//m_pointClouds.push_back(GenPointCloud(shape3, dDonut2, GRID_SIZE_X, GRID_SIZE_Y, 0.25f, 0.16f, youngMod, poisson, glm::vec2(0.f, 0.f)));
	//m_pointClouds[index].color = glm::vec3(0.f, 0.5f, 0.5f);
	//t2 = high_resolution_clock::now();
	//std::cout << "Finished generating " << m_pointClouds[index].N << " points for DemoDonut point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	//index++;

	//t1 = high_resolution_clock::now();
	//sdf::sdFunc dCircle2(sdf::DemoCircle);
	//Shape shape4(glm::vec2(50.f, 50.f), std::vector<float>{3.f});
	//m_pointClouds.push_back(GenPointCloud(shape4, dCircle2, GRID_SIZE_X, GRID_SIZE_Y, 0.25f, 0.16f, youngMod, poisson, glm::vec2(3.f, 0.f)));
	//m_pointClouds[index].color = glm::vec3(1.f, 1.f, 0.f);
	//t2 = high_resolution_clock::now();
	//std::cout << "Finished generating " << m_pointClouds[index].N << " points for DemoCircle point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	//t1 = high_resolution_clock::now();
	//index++;

	//t1 = high_resolution_clock::now();
	//sdf::sdFunc dCircle3(sdf::DemoCircle);
	//Shape shape5(glm::vec2(90.f, 30.f), std::vector<float>{7.f});
	//m_pointClouds.push_back(GenPointCloud(shape5, dCircle3, GRID_SIZE_X, GRID_SIZE_Y, 0.25f, 0.16f, youngMod, poisson, glm::vec2(3.f, 0.f)));
	//m_pointClouds[index].color = glm::vec3(0.4f, 0.3f, 0.7f);
	//t2 = high_resolution_clock::now();
	//std::cout << "Finished generating " << m_pointClouds[index].N << " points for DemoCircle point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	//t1 = high_resolution_clock::now();
	//index++;

	//sdf::sdFunc dDonut3(sdf::DemoDonut);
	//Shape shape6(glm::vec2(20.f, 80.f), std::vector<float>{1.f, 4.f});
	//m_pointClouds.push_back(GenPointCloud(shape6, dDonut3, GRID_SIZE_X, GRID_SIZE_Y, 0.25f, 0.16f, youngMod, poisson, glm::vec2(0.f, 0.f)));
	//m_pointClouds[index].color = glm::vec3(0.8f, 0.2f, 0.2f);
	//t2 = high_resolution_clock::now();
	//std::cout << "Finished generating " << m_pointClouds[index].N << " points for DemoDonut point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	//index++;

	//1.5. Test point cloud
   //for (MaterialPoint mp : m_pointClouds[0].points) {
   //	std::cout << mp.x.x << ", " << mp.x.y << std::endl;
   //}

	//m_numPointClouds = m_pointClouds.size();
	

	
	// Create the buffers for the point clouds
	//for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
	//	GLuint pointCloudSSBO;
	//	glCreateBuffers(1, &pointCloudSSBO);
	//	pointCloudPair.second->ssbo = pointCloudSSBO;
	//	glNamedBufferStorage(
	//		pointCloudPair.second->ssbo,
	//		sizeof(MaterialPoint)*pointCloudPair.second->points.size(),
	//		&(pointCloudPair.second->points.front().x.x),
	//		GL_MAP_READ_BIT
	//	);

	//}

	//for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
	//	CalculatePointCloudVolumes(pointCloudPair.first, pointCloudPair.second);
	//}

	//for (int i = 0; i < 100; i++) {
	//	MpmTimeStep(0.f);
	//}

	// ***** Useful test functions for reading the buffers *****
	//void *ptr = glMapNamedBuffer(pointCloudSSBO, GL_READ_ONLY);
	//MaterialPoint *data = static_cast<MaterialPoint*>(ptr);
	//for (int i = 0; i < 100; ++i) {
	//	std::cout << "Material Point " << i << ":" << std::endl;
	//	std::cout << data[i] << std::endl;
	//}
	//glUnmapNamedBuffer(pointCloudSSBO);
	/*void *ptr = glMapNamedBuffer(gridSSBO, GL_READ_ONLY);*/
	/*std::cout << "Reading as GridNode: \n";
	GridNode *data = static_cast<GridNode*>(ptr);
	for (int i = 0; i < GRID_SIZE_X*GRID_SIZE_Y; i++) {
		GridNode gn = data[i];
		if (gn.m != 0.f || gn.v.x != 0.f || gn.v.y != 0.f) {
			std::cout << "Grid node: [" << i / GRID_SIZE_X << ", " << i % GRID_SIZE_X << "]" << std::endl;
			std::cout << gn << std::endl;
		}
	}*/
	//std::cout << "Reading as GLfloat: \n";
	//GLfloat *test = static_cast<GLfloat*>(ptr);
	//for (int i = 0; i < GRID_SIZE_X*GRID_SIZE_Y*4; i++) {
	//	//if (test[i] != 0.f) {
	//		std::cout << "Index: " << i << std::endl;
	//		std::cout << test[i] << std::endl;
	//	//}
	//}
	/*glUnmapNamedBuffer(gridSSBO);*/
}

void mpm::MpmEngine::CalculatePointCloudVolumes(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud)
{
	using namespace std::chrono;

	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	std::cout << "Calculating initial volumes for '" << pointCloudID << "'...\n";
	
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloud->ssbo);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	t1 = high_resolution_clock::now();
	m_p2gCalcVolumes->Use();
	glDispatchCompute(G_NUM_GROUPS_X, G_NUM_GROUPS_Y, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	m_g2pCalcVolumes->Use();
	int g2p_workgroups = int(glm::ceil(float(pointCloud->N) / float(G2P_WORKGROUP_SIZE)));
	glDispatchCompute(g2p_workgroups, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	t2 = high_resolution_clock::now();
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);

	std::cout << "Finished calculating initial volumes for '" << pointCloudID << "' in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
}
