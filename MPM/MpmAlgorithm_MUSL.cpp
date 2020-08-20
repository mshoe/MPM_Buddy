#include "MpmAlgorithmEngine.h"
#include "ShapeFunctions.h"

void mpm::MpmAlgorithmEngine::MassToNode_MUSL(const mpm::MaterialPoint& mp, mpm::GridNode& node) {

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

	/*
	from MATLAB code:
	niforce(id,1)   = niforce(id,1) - pvol*(stress(1)*dNIdx + stress(3)*dNIdy);
	niforce(id,2)   = niforce(id,2) - pvol*(stress(3)*dNIdx + stress(2)*dNIdy);
	*/
	/*node.f_int -= mp.vol * (mp.stress.x * wpgSlope.x + mp.stress.z * wpgSlope.y);
	node.f_int -= mp.vol * (mp.stress.z * wpgSlope.x + mp.stress.y * wpgSlope.y);*/
	node.f_int -= mp.vol * mp.P * glm::transpose(mp.Fe) * wpgSlope;
}

void mpm::MpmAlgorithmEngine::UpdateNodeMomentum_MUSL(mpm::GridNode& node, vec2 f_ext, real dt)
{
	// actually f_ext is acceleration, just ignore for now


	if (node.m != 0.0) {

		//node.f_int += vec2(0.0, -9.81) * node.m; // gravity
		node.force = node.f_int + f_ext * node.m;

		node.momentum += dt * node.force;
	}
}

void mpm::MpmAlgorithmEngine::NodeToParticleVelocity_MUSL(const mpm::GridNode& node, mpm::MaterialPoint& mp, real dt)
{

	vec2 xp = mp.x;
	vec2 xg = node.x;
	vec2 dpg = xp - xg;
	real dx = dpg.x; // sign matters for gradient
	real dy = dpg.y;

	real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);

	if (node.m != 0.0) {
		mp.v += dt * wpg * node.force / node.m;// +dt * wpg * vec2(0.0, -9.81);
	}
}

void mpm::MpmAlgorithmEngine::VelocityToNode_MUSL(const mpm::MaterialPoint& mp, mpm::GridNode& node)
{
	vec2 xp = mp.x;
	vec2 xg = node.x;
	vec2 dpg = xp - xg;
	real dx = dpg.x; // sign matters for gradient
	real dy = dpg.y;



	real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);

	node.v += mp.v * mp.m * wpg;

}

void mpm::MpmAlgorithmEngine::CalculateNodeVelocity_MUSL(mpm::GridNode& node, double dt)
{
	if (node.m != 0.0) {
		node.v = node.v / node.m;
	}
}

void mpm::MpmAlgorithmEngine::NodeToParticlePosition_MUSL(const mpm::GridNode& node, mpm::MaterialPoint& mp, mat2& Lp, double dt)
{
	vec2 xp = mp.x;
	vec2 xg = node.x;
	vec2 dpg = xp - xg;
	real dx = dpg.x; // sign matters for gradient
	real dy = dpg.y;


	real wpg = nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunction(dy);

	vec2 wpgSlope = vec2(nodeGetter.ShapeFunctionSlope(dx) * nodeGetter.ShapeFunction(dy),
						 nodeGetter.ShapeFunction(dx) * nodeGetter.ShapeFunctionSlope(dy));



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

		vec2 dx = dt * mp.v;
		if (dx.x >= 1.0 || dx.y >= 1.0) {
			m_paused = true; // GRID CROSSING INSTABILITY
		}

		mp.x += dt * wpg * node.v;// node.momentum / node.m;
		Lp += glm::outerProduct(node.v, wpgSlope);
	}
}

void mpm::MpmAlgorithmEngine::ParticleUpdateStressStrain_MUSL(mpm::MaterialPoint& mp, real dt, ENERGY_MODEL comodel) {
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



	
	


	
	
}



void mpm::MpmAlgorithmEngine::MpmTimeStep_MUSL(real dt)
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
			m_mpmEngine->m_grid->nodes[index].nodalAcceleration = vec2(0.0);
		}
	}

	MpmTimeStepP2G_MUSL(dt);
	MpmTimeStepG_Momentum_MUSL(dt);
	MpmTimeStepG2P_Velocity_MUSL(dt);
	MpmTimeStepP2G_Velocity_MUSL(dt);
	MpmTimeStepG_Velocity_MUSL(dt);
	MpmTimeStepG2P_Position_MUSL(dt);
	MpmTimeStepP_Stress(dt);
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP2G_MUSL(real dt)
{

	/*
	1. The data from the MPs is mapped to the DOFs of the background grid.
	For instance, the diagonal of the lumped mass matrix (Eq 4.5) and the
	internal forces f^int (Eq 4.6) are computed

	*/

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


			nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

			while (!nodeGetter.Complete()) {
				GridNode& node = nodeGetter.NextNode();
				if (nodeGetter.IsNodeOK()) {

					MassToNode_MUSL(mp, node);
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG_Momentum_MUSL(real dt)
{
	for (size_t i = 0; i < size_t(m_mpmEngine->m_chunks_x) * size_t(m_cppChunkX); i++) {
		for (size_t j = 0; j < size_t(m_mpmEngine->m_chunks_y) * size_t(m_cppChunkY); j++) {

			size_t index = size_t(i) + size_t(j) * size_t(GRID_SIZE_Y);

			GridNode& node = m_mpmEngine->m_grid->nodes[index];


			
			vec2 mouseForce = m_mpmControlEngine->m_mousePower * real(m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull.w) * glm::normalize(vec2(m_mpmEngine->m_mouseMpmRenderScreenGridSpace.x - node.x.x, m_mpmEngine->m_mouseMpmRenderScreenGridSpace.y - node.x.y));
			vec2 f_ext = m_mpmControlEngine->m_globalForce + mouseForce;

			UpdateNodeMomentum_MUSL(node, f_ext, m_dt);
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG2P_Velocity_MUSL(real dt)
{

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


			nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

			while (!nodeGetter.Complete()) {
				GridNode& node = nodeGetter.NextNode();
				if (nodeGetter.IsNodeOK()) {

					NodeToParticleVelocity_MUSL(node, mp, dt);
				}
			}
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepP2G_Velocity_MUSL(real dt)
{
	// don't need to reset velocity here, as only nodal mass, force, and momentum have been computed so far

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {

			nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

			while (!nodeGetter.Complete()) {
				GridNode& node = nodeGetter.NextNode();
				if (nodeGetter.IsNodeOK()) {

					VelocityToNode_MUSL(mp, node);
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

			GridNode& node = m_mpmEngine->m_grid->nodes[index];

			CalculateNodeVelocity_MUSL(node, dt);
		}
	}
}

void mpm::MpmAlgorithmEngine::MpmTimeStepG2P_Position_MUSL(real dt)
{

	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {

		for (MaterialPoint& mp : pointCloudPair.second->points) {


			mat2 Lp = mat2(0.0);

			nodeGetter = Basis::NodeGetter(mp, m_mpmEngine->m_grid, m_basisFunction);

			while (!nodeGetter.Complete()) {
				GridNode& node = nodeGetter.NextNode();
				if (nodeGetter.IsNodeOK()) {

					NodeToParticlePosition_MUSL(node, mp, Lp, dt);
				}
			}

			mp.Fe = (mat2(1.0) + dt * Lp) * mp.Fe;
			mp.vol = glm::determinant(mp.Fe) * mp.vol0;




			/*mat2 dEps = dt * 0.5 * (Lp + glm::transpose(Lp));

			static mat3 C = mpm::LinearElasticity::ElasticityMatrix(1000, 0.3, true);

			vec3 dsigma = C * vec3(dEps[0][0], dEps[1][1], dEps[1][0] * 2.0);

			mp.stress += vec4(dsigma, 0.0);
			mp.strain += vec4(dEps[0][0], dEps[1][1], dEps[1][0] * 2.0, 0.0);*/
		}
	}
}



