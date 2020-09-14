#include "MpmAlgorithmEngine.h"
#include "ShapeFunctions.h"

void mpm::MpmAlgorithmEngine::MpmTimeStep_USF(real dt)
{
	// reset the grid
	for (size_t i = 0; i < m_mpmEngine->m_grid->grid_dim_x; i++) {
		for (size_t j = 0; j < m_mpmEngine->m_grid->grid_dim_y; j++) {
			GridNode& node = m_mpmEngine->m_grid->GetNode(i, j);
			node.m = 0.0;
			node.v = vec2(0.0);
			node.momentum = vec2(0.0);
			node.f_int = vec2(0.0);
			node.force = vec2(0.0);
			node.nodalAcceleration = vec2(0.0);
		}
	}

	MpmTimeStepP2G_Velocity_USF(dt);
	MpmTimeStepG_Velocity_USF(dt); // USF
	MpmTimeStepG2P_GradientVelocity_USF(dt);
	MpmTimeStepP_Stress(dt); // USF
	MpmTimeStepP2G_Forces_USF(dt);
	MpmTimeStepG_Momentum_MUSL(dt); // USF
	MpmTimeStepG2P_PositionVelocity_USF(dt);
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP2G_Velocity_USF(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


			mat2 Lp = mat2(0.0);

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


					node.m += wpg * mp.m;

					node.momentum += wpg * mp.m * mp.v;
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG_Velocity_USF(real dt)
{
	for (size_t i = 0; i < m_mpmEngine->m_grid->grid_dim_x; i++) {
		for (size_t j = 0; j < m_mpmEngine->m_grid->grid_dim_y; j++) {

			GridNode& node = m_mpmEngine->m_grid->GetNode(i, j);
			if (node.m != 0.0) {
				node.v = node.momentum / node.m;
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG2P_GradientVelocity_USF(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


			mat2 Lp = mat2(0.0);

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

					Lp += glm::outerProduct(node.v, wpgSlope);
				}
			}

			mp.Fe = (mat2(1.0) + dt * Lp) * mp.Fe;
			mp.vol = glm::determinant(mp.Fe) * mp.vol0;
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP2G_Forces_USF(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


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

					node.f_int -= mp.vol * mp.P * glm::transpose(mp.Fe) * wpgSlope;
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG_Momentum_USF(real dt)
{
	for (size_t i = 0; i < m_mpmEngine->m_grid->grid_dim_x; i++) {
		for (size_t j = 0; j < m_mpmEngine->m_grid->grid_dim_y; j++) {
			size_t index = i + m_mpmEngine->m_grid->grid_dim_y * j;

			GridNode& node = m_mpmEngine->m_grid->GetNode(i, j);



			vec2 mouseForce = m_mpmControlEngine->m_mousePower * real(m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull.w) * glm::normalize(vec2(m_mpmEngine->m_mouseMpmRenderScreenGridSpace.x - node.x.x, m_mpmEngine->m_mouseMpmRenderScreenGridSpace.y - node.x.y));
			vec2 f_ext = m_mpmControlEngine->m_globalForce + mouseForce;

			UpdateNodeMomentum_MUSL(node, f_ext, m_dt);
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG2P_PositionVelocity_USF(real dt)
{
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


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


					if (node.m != 0.0) {

						vec2 dx = dt * mp.v;
						if (dx.x >= 0.5 || dx.y >= 0.5) {
							m_paused = true; // APPROACHING GRID CROSSING INSTABILITY
						}

						mp.v += dt * wpg * node.force / node.m;
						mp.x += dt * wpg * node.momentum / node.m;
					}
				}
			}
		}
	}
}
