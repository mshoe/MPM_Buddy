#include "MpmAlgorithmEngine.h"
#include "ShapeFunctions.h"

void mpm::MpmAlgorithmEngine::MpmTimeStep_SE(real dt)
{
	// reset the grid
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {
			size_t index = i + GRID_SIZE_Y * j;
			m_mpmEngine->m_grid->nodes[index].m = 0.0;
			m_mpmEngine->m_grid->nodes[index].v = vec2(0.0);
			m_mpmEngine->m_grid->nodes[index].momentum = vec2(0.0);
			m_mpmEngine->m_grid->nodes[index].f_int = vec2(0.0);
			m_mpmEngine->m_grid->nodes[index].force = vec2(0.0);
		}
	}

	MpmTimeStepP_Stress(dt);
	MpmTimeStepP2G_SE(dt);
	MpmTimeStepG_Velocity_SE(dt);
	MpmTimeStepG2P_SE(dt);


}

void mpm::MpmAlgorithmEngine::MpmTimeStepP2G_SE(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {

			

			nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

			while (!nodeGetter.Complete()) {
				GridNode& node = nodeGetter.NextNode();
				if (nodeGetter.IsNodeOK()) {

					vec2 xg = node.x;
					vec2 dpg = xg - mp.x;
					real dx = dpg.x; // sign matters for gradient
					real dy = dpg.y;
					real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);
					vec2 wpgSlope = vec2(nodeGetter.ShapeFunctionSlope(dx) * nodeGetter.ShapeFunction(dy),
										 nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunctionSlope(dy));

					// P2G mass/momentum transfer
					node.m += wpg * mp.m;
					node.momentum += wpg * mp.m * mp.v;
					node.f_int += mp.vol0 * mp.P * glm::transpose(mp.Fe) * wpgSlope; // WHY DOES THIS ONLY WORK WHEN + BUT NOT -?
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG_Velocity_SE(real dt)
{
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {

			size_t index = size_t(i) + size_t(j) * size_t(GRID_SIZE_Y);
			GridNode& node = m_mpmEngine->m_grid->nodes[index];
			

			real nodeMass = node.m;

			if (nodeMass == 0.0)
				continue;

			vec2 xg = node.x;
			vec2 nodeMomentum = node.momentum;
			vec2 mouseAcc = m_mpmControlEngine->m_mousePower * real(m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull.w) * glm::normalize(vec2(m_mpmEngine->m_mouseMpmRenderScreenGridSpace.x - xg.x, m_mpmEngine->m_mouseMpmRenderScreenGridSpace.y - xg.y));
			
			vec2 gridV = nodeMomentum / nodeMass;

			node.force = node.f_int + (mouseAcc + m_mpmControlEngine->m_globalForce) * node.m;

			vec2 gridUpdateV = gridV  + dt * node.force / node.m;
			node.v = gridUpdateV;
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG2P_SE(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {

			mat2 Lp = mat2(0.0);

			vec2 v_flip = mp.v;
			vec2 v_pic = vec2(0.0);

			double alpha = 0.95;

			nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

			while (!nodeGetter.Complete()) {
				GridNode& node = nodeGetter.NextNode();
				if (nodeGetter.IsNodeOK()) {

					vec2 xp = mp.x;
					vec2 xg = node.x;
					vec2 dpg = xp - xg;
					real dx = dpg.x; // sign matters for gradient
					real dy = dpg.y;


					real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);

					vec2 wpgSlope = vec2(nodeGetter.ShapeFunctionSlope(dx) * nodeGetter.ShapeFunction(dy),
										 nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunctionSlope(dy));


					if (node.m != 0.0) {

						Lp += glm::outerProduct(node.v, wpgSlope);

						v_flip += dt * node.force / node.m * wpg;
						v_pic += node.v * wpg;
					}
				}
			}

			mp.Fe = (mat2(1.0) + dt * Lp) * mp.Fe;
			mp.v = (1.0 - alpha) * v_pic + alpha * v_flip;
			
			vec2 dx = dt * mp.v;
			if (dx.x >= 1.0 || dx.y >= 1.0) {
				m_paused = true; // GRID CROSSING INSTABILITY
			}

			mp.x += dt * mp.v;

		}
	}
}