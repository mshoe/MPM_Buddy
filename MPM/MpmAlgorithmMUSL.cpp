#include "MpmAlgorithmEngine.h"

// helper functions to make code cleaner
void MassToNode(const mpm::MaterialPoint& mp, mpm::GridNode& node);
void UpdateNodeMomentum(mpm::GridNode& node, vec2 f_ext, real dt);
void NodeToParticleVelocity(const mpm::GridNode& node, mpm::MaterialPoint& mp, real dt);
void VelocityToNode(const mpm::MaterialPoint& mp, mpm::GridNode& node);
void CalculateNodeVelocity(mpm::GridNode& node);

void MassToNode(const mpm::MaterialPoint& mp, mpm::GridNode& node) {

	vec2 xp = mp.x;
	vec2 xg = node.x;
	vec2 dpg = xp - xg;
	real dx = dpg.x; // sign matters for gradient
	real dy = dpg.y;



	// real wpg = mpm::LinearShape(dx) * mpm::LinearShape(dy);
	real wpg = mpm::LinearShape(dx) * mpm::LinearShape(dy);

	vec2 wpgSlope = vec2(mpm::LinearShapeSlope(dx) * mpm::LinearShape(dy),
						  mpm::LinearShape(dx) * mpm::LinearShapeSlope(dy));


	node.m += wpg * mp.m;

	node.momentum += wpg * mp.m * mp.v;

	/*
	from MATLAB code:
	niforce(id,1)   = niforce(id,1) - pvol*(stress(1)*dNIdx + stress(3)*dNIdy);
	niforce(id,2)   = niforce(id,2) - pvol*(stress(3)*dNIdx + stress(2)*dNIdy);
	*/
	node.f_int -= mp.vol * (mp.stress.x * wpgSlope.x + mp.stress.z * wpgSlope.y);
	node.f_int -= mp.vol * (mp.stress.z * wpgSlope.x + mp.stress.y * wpgSlope.y);
}

void UpdateNodeMomentum(mpm::GridNode& node, vec2 f_ext, real dt)
{
	// actually f_ext is acceleration, just ignore for now

	if (node.m != 0.0) {
		node.momentum += dt * node.f_int;// +dt * f_ext * node.m;
	}
}

void NodeToParticleVelocity(const mpm::GridNode& node, mpm::MaterialPoint& mp, real dt)
{

	vec2 xp = mp.x;
	vec2 xg = node.x;
	vec2 dpg = xp - xg;
	real dx = dpg.x; // sign matters for gradient
	real dy = dpg.y;



	// real wpg = mpm::LinearShape(dx) * mpm::LinearShape(dy);
	real wpg = mpm::LinearShape(dx) * mpm::LinearShape(dy);

	/*vec2 wpgSlope = -vec2(mpm::LinearShapeSlope(dx) * mpm::LinearShape(dy),
						  mpm::LinearShape(dx) * mpm::LinearShapeSlope(dy));*/

	if (node.m != 0.0) {
		mp.v += dt * wpg * node.f_int / node.m;
	}
}

void VelocityToNode(const mpm::MaterialPoint& mp, mpm::GridNode& node)
{
	vec2 xp = mp.x;
	vec2 xg = node.x;
	vec2 dpg = xp - xg;
	real dx = dpg.x; // sign matters for gradient
	real dy = dpg.y;



	// real wpg = mpm::LinearShape(dx) * mpm::LinearShape(dy);
	real wpg = mpm::LinearShape(dx) * mpm::LinearShape(dy);

	node.v += mp.v * mp.m * wpg;

}

void CalculateNodeVelocity(mpm::GridNode& node)
{
	if (node.m != 0.0) {
		node.v = node.v / node.m;
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStep_MUSL(real dt)
{
	// reset the grid
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {
			size_t index = i + GRID_SIZE_Y * j;
			m_mpmEngine->m_grid.nodes[index].m = 0.0;
			m_mpmEngine->m_grid.nodes[index].v = vec2(0.0);
			m_mpmEngine->m_grid.nodes[index].momentum = vec2(0.0);
			m_mpmEngine->m_grid.nodes[index].f_int = vec2(0.0);
			//m_mpmEngine->m_grid.nodes[index].nodalAcceleration = vec2(0.0);
		}
	}

	MpmTimeStepP2G_MUSL(dt);
	MpmTimeStepG_Update_MUSL(dt);
	MpmTimeStepG2P_Velocity_MUSL(dt);
	MpmTimeStepP2G_Velocity_MUSL(dt);
	MpmTimeStepG_Velocity_MUSL(dt);
	MpmTimeStepG2P_Position_MUSL(dt);
}





void mpm::MpmAlgorithmEngine::MpmTimeStepP2G_MUSL(real dt)
{

	/*
	1. The data from the MPs is mapped to the DOFs of the background grid.
	For instance, the diagonal of the lumped mass matrix (Eq 4.5) and the
	internal forces f^int (Eq 4.6) are computed

	*/

	int x_bound = m_mpmEngine->m_chunks_x * m_cppChunkX;
	int y_bound = m_mpmEngine->m_chunks_y * m_cppChunkY;

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


			int botLeftNode_i = int(glm::floor(mp.x.x));
			int botLeftNode_j = int(glm::floor(mp.x.y));

			for (int i = 0; i <= 1; i++) {
				for (int j = 0; j <= 1; j++) {

					int currNode_i = botLeftNode_i + i;
					int currNode_j = botLeftNode_j + j;

					if (!InBounds(currNode_i, currNode_j, x_bound, y_bound)) {
						continue;
					}

					

					size_t index = size_t(currNode_i) + size_t(currNode_j) * size_t(GRID_SIZE_Y);

					GridNode& node = m_mpmEngine->m_grid.nodes[index];

					MassToNode(mp, node);
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG_Update_MUSL(real dt)
{
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {

			size_t index = size_t(i) + size_t(j) * size_t(GRID_SIZE_Y);

			GridNode& node = m_mpmEngine->m_grid.nodes[index];


			
			vec2 mouseForce = m_mpmControlEngine->m_mousePower * real(m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull.w) * glm::normalize(vec2(m_mpmEngine->m_mouseMpmRenderScreenGridSpace.x - node.x.x, m_mpmEngine->m_mouseMpmRenderScreenGridSpace.y - node.x.y));
			vec2 f_ext = m_mpmControlEngine->m_globalForce + mouseForce;

			UpdateNodeMomentum(node, f_ext, m_dt);
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG2P_Velocity_MUSL(real dt)
{
	int x_bound = m_mpmEngine->m_chunks_x * m_cppChunkX;
	int y_bound = m_mpmEngine->m_chunks_y * m_cppChunkY;

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


			int botLeftNode_i = int(glm::floor(mp.x.x));
			int botLeftNode_j = int(glm::floor(mp.x.y));

			for (int i = 0; i <= 1; i++) {
				for (int j = 0; j <= 1; j++) {

					int currNode_i = botLeftNode_i + i;
					int currNode_j = botLeftNode_j + j;

					if (!InBounds(currNode_i, currNode_j, x_bound, y_bound)) {
						continue;
					}



					size_t index = size_t(currNode_i) + size_t(currNode_j) * size_t(GRID_SIZE_Y);

					GridNode& node = m_mpmEngine->m_grid.nodes[index];

					NodeToParticleVelocity(node, mp, dt);
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP2G_Velocity_MUSL(real dt)
{
	int x_bound = m_mpmEngine->m_chunks_x * m_cppChunkX;
	int y_bound = m_mpmEngine->m_chunks_y * m_cppChunkY;

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


			int botLeftNode_i = int(glm::floor(mp.x.x));
			int botLeftNode_j = int(glm::floor(mp.x.y));

			for (int i = 0; i <= 1; i++) {
				for (int j = 0; j <= 1; j++) {

					int currNode_i = botLeftNode_i + i;
					int currNode_j = botLeftNode_j + j;

					if (!InBounds(currNode_i, currNode_j, x_bound, y_bound)) {
						continue;
					}



					size_t index = size_t(currNode_i) + size_t(currNode_j) * size_t(GRID_SIZE_Y);

					GridNode& node = m_mpmEngine->m_grid.nodes[index];

					VelocityToNode(mp, node);
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG_Velocity_MUSL(real dt)
{
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {

			size_t index = size_t(i) + size_t(j) * size_t(GRID_SIZE_Y);

			GridNode& node = m_mpmEngine->m_grid.nodes[index];

			CalculateNodeVelocity(node);
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG2P_Position_MUSL(real dt)
{
	

	int x_bound = m_mpmEngine->m_chunks_x * m_cppChunkX;
	int y_bound = m_mpmEngine->m_chunks_y * m_cppChunkY;

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


			mat2 Lp = mat2(0.0);

			int botLeftNode_i = int(glm::floor(mp.x.x));
			int botLeftNode_j = int(glm::floor(mp.x.y));

			for (int i = 0; i <= 1; i++) {
				for (int j = 0; j <= 1; j++) {

					int currNode_i = botLeftNode_i + i;
					int currNode_j = botLeftNode_j + j;

					if (!InBounds(currNode_i, currNode_j, x_bound, y_bound)) {
						continue;
					}



					size_t index = size_t(currNode_i) + size_t(currNode_j) * size_t(GRID_SIZE_Y);

					GridNode& node = m_mpmEngine->m_grid.nodes[index];

					vec2 xp = mp.x;
					vec2 xg = node.x;
					vec2 dpg = xp - xg;
					real dx = dpg.x; // sign matters for gradient
					real dy = dpg.y;



					// real wpg = mpm::LinearShape(dx) * mpm::LinearShape(dy);
					real wpg = mpm::LinearShape(dx) * mpm::LinearShape(dy);

					vec2 wpgSlope = vec2(mpm::LinearShapeSlope(dx) * mpm::LinearShape(dy),
										  mpm::LinearShape(dx) * mpm::LinearShapeSlope(dy));



					/*
					[N, dNdxi] = lagrange_basis('Q4', pt);% element shape functions
					J0 = enode'*dNdxi;             % element Jacobian matrix
					invJ0 = inv(J0);
					dNdx = dNdxi * invJ0;
					Lp = zeros(2, 2);
					for i = 1:length(esctr)
						id = esctr(i);
						vI = nvelo(id, :);
						xp(pid, :) = xp(pid, :) + dtime * N(i) * nmomentum(id, :) / nmass(id);
						Lp = Lp + vI'*dNdx(i,:);         % particle gradient velocity 
					end
					*/

					if (node.m != 0.0) {
						mp.x += dt * wpg * node.momentum / node.m;
						Lp += glm::outerProduct(node.v, wpgSlope);
					}

				}
			}

			/*
			F       = ([1 0;0 1] + Lp*dtime)*reshape(Fp(pid,:),2,2);
			Fp(pid,:)= reshape(F,1,4);
			Vp(pid) = det(F)*Vp0(pid);
			dEps = dtime * 0.5 * (Lp + Lp');
			dsigma = C * [dEps(1, 1); dEps(2, 2); 2 * dEps(1, 2)];
			s(pid, :) = s(pid, :) + dsigma';
			eps(pid, :) = eps(pid, :) + [dEps(1, 1) dEps(2, 2) 2 * dEps(1, 2)];

			k = k + 0.5 * (vp(pid, 1) ^ 2 + vp(pid, 2) ^ 2) * Mp(pid);
			u = u + 0.5 * Vp(pid) * s(pid, :) * eps(pid, :)';
			*/



			mp.Fe = (mat2(1.0) + dt * Lp) * mp.Fe;
			mp.vol = glm::determinant(mp.Fe) * mp.vol0;
			mat2 dEps = dt * 0.5 * (Lp + glm::transpose(Lp));

			static mat3 C = ElasticityMatrix(1000000.0, 0.3, true);

			vec3 dsigma = C * vec3(dEps[0][0], dEps[1][1], dEps[1][0] * 2.0);

			mp.stress += vec4(dsigma, 0.0);
			mp.strain += vec4(dEps[0][0], dEps[1][1], dEps[1][0] * 2.0, 0.0);

			//vec3 dsigma = 

		}
	}
}
