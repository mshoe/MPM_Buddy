#include "MpmAlgorithmEngine.h"

#include "MpmFunctions.h"
#include "EnergyFunctions.h"



void mpm::MpmAlgorithmEngine::RunMPMSimulationCPP(real dt, size_t num_steps, bool debugOutput, bool renderAfter)
{
	if (debugOutput)
		std::cout << "Starting MPM Simulation, CPP mode, number of time steps = " << num_steps << std::endl;
	for (size_t k = 0; k < num_steps; k++) {
		if (debugOutput)
			std::cout << "Time step: " << k << std::endl;
		MpmTimeStep_CPP(dt);
	}
	if (renderAfter)
		m_mpmEngine->MapCPUPointCloudsToGPU();
}

void mpm::MpmAlgorithmEngine::MpmReset_CPP()
{
	m_mpmEngine->m_pointCloudMap.clear();
	m_timeStep = 0;
	m_time = 0.0;

	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {
			size_t index = i * GRID_SIZE_Y + j;
			m_mpmEngine->m_grid.nodes[index].m = 0.0;
			m_mpmEngine->m_grid.nodes[index].v = vec2(0.0);
			m_mpmEngine->m_grid.nodes[index].momentum = vec2(0.0);
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStep_CPP(real dt)
{
#ifdef MPM_CPP_DEBUG
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();
#endif
	MpmTimeStepP2G_CPP(dt);

	MpmTimeStepExplicitGridUpdate_CPP(dt);

	if (m_semiImplicitCPP) {
		MpmTimeStepSemiImplicitGridUpdate_CPP(dt, m_beta);
	}

	MpmTimeStepG2P_CPP(dt);

	m_timeStep++;
	m_time += m_dt;
#ifdef MPM_CPP_DEBUG
	t2 = high_resolution_clock::now();
	std::cout << "Finished calculating timestep in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
#endif
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP2G_CPP(real dt)
{
	// reset the grid
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {
			size_t index = i * GRID_SIZE_Y + j;
			m_mpmEngine->m_grid.nodes[index].m = 0.0;
			m_mpmEngine->m_grid.nodes[index].v = vec2(0.0);
			m_mpmEngine->m_grid.nodes[index].momentum = vec2(0.0);
		}
	}

	int x_bound = m_mpmEngine->m_chunks_x * m_cppChunkX;
	int y_bound = m_mpmEngine->m_chunks_y * m_cppChunkY;

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint &point : pointCloudPair.second->points) {


			int botLeftNode_i = int(glm::floor(point.x.x)) - 1;
			int botLeftNode_j = int(glm::floor(point.x.y)) - 1;

			vec2 xp = point.x;

			mat2 P = FixedCorotationalElasticity::PKTensor(point.Fe, point.lam, point.mew);
			
			// just store it for possible debugging purposes
			point.P = P;

			for (int i = 0; i <= 3; i++) {
				for (int j = 0; j <= 3; j++) {

					int currNode_i = botLeftNode_i + i;
					int currNode_j = botLeftNode_j + j;

					if (!InBounds(currNode_i, currNode_j, x_bound, y_bound)) {
						continue;
					}

					vec2 xg = vec2(real(currNode_i), real(currNode_j));
					vec2 dpg = xg - xp;
					real dx = -dpg.x; // sign matters for gradient
					real dy = -dpg.y;
					real wpg = CubicBSpline(dx) * CubicBSpline(dy);
					
					size_t index = size_t(currNode_i) * size_t(GRID_SIZE_Y) + size_t(currNode_j);

					// P2G mass transfer
					m_mpmEngine->m_grid.nodes[index].m += wpg * point.m;

					// P2G APIC momentum transfer
					mat2 Gp = -Dp_inv * dt * point.vol * point.P * glm::transpose(point.Fe) + point.m * point.B;
					m_mpmEngine->m_grid.nodes[index].momentum += wpg * (point.m * point.v + Gp * dpg);
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepExplicitGridUpdate_CPP(real dt)
{
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {
			size_t index = i * GRID_SIZE_Y + j;

			real nodeMass = m_mpmEngine->m_grid.nodes[index].m;

			if (nodeMass == 0.0)
				continue;

			vec2 xg = vec2(real(i), real(j));
			vec2 nodeMomentum = m_mpmEngine->m_grid.nodes[index].momentum;
			vec2 mouseForce = m_mpmControlEngine->m_mousePower * real(m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull.w) * glm::normalize(vec2(m_mpmEngine->m_mouseMpmRenderScreenGridSpace.x - xg.x, m_mpmEngine->m_mouseMpmRenderScreenGridSpace.y - xg.y));
			// ignoring (experimental) nodal acceleration
			
			vec2 gridV = nodeMomentum / nodeMass;

			vec2 gridUpdateV = gridV * (1.0 - dt * m_mpmControlEngine->m_drag) + dt * (m_mpmControlEngine->m_globalForce + mouseForce);
			m_mpmEngine->m_grid.nodes[index].v = gridUpdateV;

		}
	}
}



void mpm::MpmAlgorithmEngine::MpmTimeStepG2P_CPP(real dt)
{
	int x_bound = m_mpmEngine->m_chunks_x * m_cppChunkX;
	int y_bound = m_mpmEngine->m_chunks_y * m_cppChunkY;

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint &point : pointCloudPair.second->points) {


			int botLeftNode_i = int(glm::floor(point.x.x)) - 1;
			int botLeftNode_j = int(glm::floor(point.x.y)) - 1;

			vec2 xp = point.x;

			vec2 vp = vec2(0.0);
			mat2 bp = mat2(0.0);
			

			for (int i = 0; i <= 3; i++) {
				for (int j = 0; j <= 3; j++) {

					int currNode_i = botLeftNode_i + i;
					int currNode_j = botLeftNode_j + j;

					if (!InBounds(currNode_i, currNode_j, x_bound, y_bound)) {
						continue;
					}

					vec2 xg = vec2(real(currNode_i), real(currNode_j));
					vec2 dpg = xg - xp;
					real dx = -dpg.x; // sign matters for gradient
					real dy = -dpg.y;
					real wpg = CubicBSpline(dx) * CubicBSpline(dy);

					size_t index = size_t(currNode_i) * size_t(GRID_SIZE_Y) + size_t(currNode_j);

					vec2 vg = m_mpmEngine->m_grid.nodes[index].v;

					// for material point velocity update
					vp += wpg * vg;

					// APIC
					bp += wpg * outerProduct(vg, dpg);
				}
			}

			point.v = vp;
			point.B = Dp_inv * bp; // for APIC
			point.x += dt * vp; // update position
			mat2 dfp = mat2(1.0) + dt * point.B; // for MLS
			point.Fe = dfp * point.Fe; // update deformation gradient
		}
	}
}



void mpm::MpmAlgorithmEngine::GetPointCloudVolumesFromGPUtoCPU(std::string pointCloudID, std::shared_ptr<PointCloud> pointCloud)
{
	void* ptr = glMapNamedBuffer(pointCloud->ssbo, GL_READ_ONLY);
	MaterialPoint* data = static_cast<MaterialPoint*>(ptr);

	for (size_t i = 0; i < pointCloud->N; i++) {
		pointCloud->points[i].vol = data[i].vol;
	}

	glUnmapNamedBuffer(pointCloud->ssbo);
}
