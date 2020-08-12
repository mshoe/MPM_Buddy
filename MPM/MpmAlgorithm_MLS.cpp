#include "MpmAlgorithmEngine.h"

#include "MpmFunctions.h"
#include "EnergyFunctions.h"

void mpm::MpmAlgorithmEngine::MassMomentumToNode_MLS(const mpm::MaterialPoint& mp, mpm::GridNode& node, real dt)
{
	vec2 xg = node.x;
	vec2 dpg = xg - mp.x;
	real dx = dpg.x; // sign matters for gradient
	real dy = dpg.y;
	real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);


	// P2G mass transfer
	node.m += wpg * mp.m;

	// P2G APIC momentum transfer
	mat2 Gp = -Dp_inv * dt * mp.vol * mp.P * glm::transpose(mp.Fe) + mp.m * mp.B;
	node.momentum += wpg * (mp.m * mp.v + Gp * dpg);
}

void mpm::MpmAlgorithmEngine::UpdateNodeVelocity_MLS(mpm::GridNode& node, real dt)
{

	real nodeMass = node.m;

	if (nodeMass == 0.0)
		return;

	vec2 xg = node.x;
	vec2 nodeMomentum = node.momentum;
	vec2 mouseForce = m_mpmControlEngine->m_mousePower * real(m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull.w) * glm::normalize(vec2(m_mpmEngine->m_mouseMpmRenderScreenGridSpace.x - xg.x, m_mpmEngine->m_mouseMpmRenderScreenGridSpace.y - xg.y));
	// ignoring (experimental) nodal acceleration

	vec2 gridV = nodeMomentum / nodeMass;

	vec2 gridUpdateV = gridV * (1.0 - dt * m_mpmControlEngine->m_drag) + dt * (m_mpmControlEngine->m_globalForce + mouseForce);
	node.v = gridUpdateV;
}

void mpm::MpmAlgorithmEngine::VelocityToParticle_MLS(const mpm::GridNode& node, mpm::MaterialPoint& mp, real dt)
{
	vec2 xg = node.x;
	vec2 dpg = xg - mp.x;
	real dx = dpg.x; // sign matters for gradient
	real dy = dpg.y;
	real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);

	vec2 vg = node.v;

	// for material point velocity update
	mp.v += wpg * vg;

	// APIC
	mp.B += wpg * outerProduct(vg, dpg);
}



void mpm::MpmAlgorithmEngine::RunMPMSimulationMLS(real dt, size_t num_steps, bool debugOutput, bool renderAfter)
{
	if (debugOutput)
		std::cout << "Starting MPM Simulation, CPP mode, number of time steps = " << num_steps << std::endl;
	for (size_t k = 0; k < num_steps; k++) {
		if (debugOutput)
			std::cout << "Time step: " << k << std::endl;
		MpmTimeStep_MLS(dt);
	}
	if (renderAfter)
		m_mpmEngine->MapCPUPointCloudsToGPU();
}

void mpm::MpmAlgorithmEngine::MpmReset_MLS()
{
	m_mpmEngine->m_pointCloudMap.clear();
	m_timeStep = 0;
	m_time = 0.0;

	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {
			size_t index = i * GRID_SIZE_Y + j;
			m_mpmEngine->m_grid->nodes[index].m = 0.0;
			m_mpmEngine->m_grid->nodes[index].v = vec2(0.0);
			m_mpmEngine->m_grid->nodes[index].momentum = vec2(0.0);
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStep_MLS(real dt)
{
#ifdef MPM_CPP_DEBUG
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();
#endif

	// reset the grid
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {
			size_t index = i * GRID_SIZE_Y + j;
			m_mpmEngine->m_grid->nodes[index].m = 0.0;
			m_mpmEngine->m_grid->nodes[index].v = vec2(0.0);
			m_mpmEngine->m_grid->nodes[index].momentum = vec2(0.0);
		}
	}

	if (m_USL) {
		MpmTimeStepP1_MLS(dt);
	}

	MpmTimeStepP2G_MLS(dt);

	MpmTimeStepExplicitGridUpdate_MLS(dt);

	if (m_semiImplicitCPP) {
		MpmTimeStepSemiImplicitGridUpdate_MLS(dt, m_beta);
	}

	if (!m_USL) {
		MpmTimeStepP1_MLS(dt);
	}

	MpmTimeStepG2P_MLS(dt);

	MpmTimeStepP2_MLS(dt);

	m_timeStep++;
	m_time += m_dt;
#ifdef MPM_CPP_DEBUG
	t2 = high_resolution_clock::now();
	std::cout << "Finished calculating timestep in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
#endif
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP1_MLS(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& point : pointCloudPair.second->points) {

			switch (pointCloudPair.second->comodel) {
			case ENERGY_MODEL::LINEAR_ELASTICITY:
				point.P = LinearElasticity::PKTensor(point.Fe, point.lam, point.mew);
				break;
			case ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY:
				point.P = NeoHookean::PKTensor(point.Fe, point.lam, point.mew);
				break;
			case ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY:
				point.P = FixedCorotationalElasticity::PKTensor(point.Fe, point.lam, point.mew);
				break;
			case ENERGY_MODEL::SIMPLE_SNOW:
				point.P = SimpleSnow::PKTensor(point.Fe, point.Fp, point.lam, point.mew, point.crit_c, point.crit_s, point.hardening);
				break;
			default:
				break;
			}

			
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP2G_MLS(real dt)
{

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint &mp : pointCloudPair.second->points) {


			nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

			while (!nodeGetter.Complete()) {
				GridNode& node = nodeGetter.NextNode();
				if (nodeGetter.IsNodeOK()) {

					MassMomentumToNode_MLS(mp, node, dt);
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepExplicitGridUpdate_MLS(real dt)
{
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {

			size_t index = size_t(i) + size_t(j) * size_t(GRID_SIZE_Y);
			GridNode& node = m_mpmEngine->m_grid->nodes[index];
			UpdateNodeVelocity_MLS(node, dt);
		}
	}
}



void mpm::MpmAlgorithmEngine::MpmTimeStepG2P_MLS(real dt)
{
	int x_bound = m_mpmEngine->m_chunks_x * m_cppChunkX;
	int y_bound = m_mpmEngine->m_chunks_y * m_cppChunkY;

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint &mp : pointCloudPair.second->points) {


			mp.v = vec2(0.0);
			mp.B = mat2(0.0);
			

			nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

			while (!nodeGetter.Complete()) {
				GridNode& node = nodeGetter.NextNode();
				if (nodeGetter.IsNodeOK()) {

					VelocityToParticle_MLS(node, mp, dt);
				}
			}

			mp.B = Dp_inv * mp.B; // for APIC
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP2_MLS(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {
			
			vec2 dx = dt * mp.v;
			if (dx.x >= 1.0 || dx.y >= 1.0) {
				m_paused = true; // GRID CROSSING INSTABILITY
			}

			mp.x += dt * mp.v; // update position
			mat2 dfp = mat2(1.0) + dt * mp.B; // for MLS
			mp.Fe = dfp * mp.Fe; // update deformation gradient
		}
	}

	if (m_paused) {
		std::cout << "grid crossing instability, simulation paused" << std::endl;
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
