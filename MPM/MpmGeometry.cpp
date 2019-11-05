#include "MpmEngine.h"

void mpm::MpmEngine::GenPointCloudPolygon()
{
	if (!m_paused)
		return;

	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();




	std::string polygonID = "polygon" + std::to_string(m_polygonCount);
	m_polygonCount++;

	glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

	std::shared_ptr<PointCloud> pointCloud = GenPointCloud(polygonID, *m_polygon, real(m_chunks_x) * real(CHUNK_WIDTH), real(m_chunks_y) * real(CHUNK_WIDTH), 0.0, 0.0, m_particleSpacing, m_mpParameters, m_comodel, sdf::SDF_OPTION::NORMAL, m_invertedSdf, m_fixedPointCloud, m_initVelocity, color);

	t2 = high_resolution_clock::now();

	std::cout << "Finished generating " << pointCloud->N << " points for '" << polygonID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";

}

void mpm::MpmEngine::GenPointCloudPWLine()
{
	if (!m_paused)
		return;

	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();




	std::string pwLineID = "line" + std::to_string(m_pwLineCount);
	m_pwLineCount++;

	glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

	std::shared_ptr<PointCloud> pointCloud = GenPointCloud(pwLineID, *m_pwLine, real(m_chunks_x) * real(CHUNK_WIDTH), real(m_chunks_y) * real(CHUNK_WIDTH), 0.0, m_pwLineRounding, m_particleSpacing, m_mpParameters, m_comodel, sdf::SDF_OPTION::ROUNDED, false, m_fixedPointCloud, m_initVelocity, color);

	t2 = high_resolution_clock::now();

	std::cout << "Finished generating " << pointCloud->N << " points for '" << pwLineID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
}

std::shared_ptr<mpm::PointCloud> mpm::MpmEngine::GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
															   const real gridDimX, const real gridDimY,
															   const real inner_rounding, const real outer_rounding,
															   const real particle_spacing, const MaterialParameters& parameters,
															   const ENERGY_MODEL comodel, enum class sdf::SDF_OPTION sdfOption,
															   bool inverted, bool fixed,
															   vec2 initialVelocity, glm::highp_fvec4 color)
{
	std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();

	pointCloud->color = color;
	pointCloud->parameters = parameters;
	pointCloud->parameters.CalculateLameParameters();/*
	pointCloud->mew = parameters.youngMod / (2.f + 2.f* parameters.poisson);
	pointCloud->lam = parameters.youngMod * parameters.poisson / ((1.f + parameters.poisson) * (1.f - 2.f * parameters.poisson));*/

	std::cout << "mew: " << pointCloud->parameters.mew << ", lam: " << pointCloud->parameters.lam << std::endl;
	//m_mew = pointCloud->mew;
	//m_lam = pointCloud->lam;

	pointCloud->comodel = comodel;

	pointCloud->fixed = fixed;

	real mass = particle_spacing * particle_spacing * parameters.density;

	// gen points from sdf
	for (real x = 0.0; x < gridDimX; x += particle_spacing) {
		for (real y = 0.0; y < gridDimY; y += particle_spacing) {

			glm::vec2 p(x, y);

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
				MaterialPoint mp(p, initialVelocity, GLreal(mass));
				// calculate mp.vol in a compute shader (not here)


				pointCloud->points.push_back(mp);
			}
		}
	}
	pointCloud->N = pointCloud->points.size();

	if (pointCloud->N > 0) {

		// Create the SSBO for the point cloud, so it is stored on the GPU
		GLuint pointCloudSSBO;
		glCreateBuffers(1, &pointCloudSSBO);
		pointCloud->ssbo = pointCloudSSBO;
		glNamedBufferStorage(
			pointCloud->ssbo,
			sizeof(MaterialPoint) * pointCloud->points.size(),
			&(pointCloud->points.front().x.x),
			GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // add write bit for cpu mode
		);

		// Calculate volumes for the point cloud (volumes stored in SSBO on GPU)
		CalculatePointCloudVolumes_GLSL(pointCloudID, pointCloud);

		if (m_algo_code == MPM_ALGORITHM_CODE::CPP) {
			// for CPU mode calculate volumes
			GetPointCloudVolumesFromGPUtoCPU(pointCloudID, pointCloud);
		}


		m_pointCloudMap[pointCloudID] = pointCloud;
	}

	return pointCloud;
}


void mpm::MpmEngine::HandleGeometryStates()
{
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	if (m_paused && m_rightButtonDown) {
		ClearCreateStates();
	}

	if (m_paused && m_createCircleState && m_leftButtonDown)
	{
		m_createCircleState = false;

		std::cout << "Mouse position is at (" << m_mouseMpmRenderScreenGridSpace.x << ", " << m_mouseMpmRenderScreenGridSpace.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_circleCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Circle shape(m_mouseMpmRenderScreenGridSpace, m_circle_r);
		std::string circleID = "circle" + std::to_string(m_circleCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		/*color.x = (float)glm::clamp(m_color[0], 0, 255) / 255.f;
		color.y = (float)glm::clamp(m_color[1], 0, 255) / 255.f;
		color.z = (float)glm::clamp(m_color[2], 0, 255) / 255.f;*/

		real inner_rounding = m_circle_r - m_circle_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(circleID, shape, real(m_chunks_x) * real(CHUNK_WIDTH), real(m_chunks_y) * real(CHUNK_WIDTH), inner_rounding, m_circle_rounding, m_particleSpacing, m_mpParameters, m_comodel, sdf::SDF_OPTION::HOLLOW, false, m_fixedPointCloud, m_initVelocity, color);

		t2 = high_resolution_clock::now();



		std::cout << "Finished generating " << pointCloud->N << " points for '" << circleID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_createRectState && m_leftButtonDown)
	{
		m_createRectState = false;

		std::cout << "Mouse position is at (" << m_mouseMpmRenderScreenGridSpace.x << ", " << m_mouseMpmRenderScreenGridSpace.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_rectCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Rectangle shape(m_mouseMpmRenderScreenGridSpace, m_rect_b, m_rect_h);
		std::string rectID = "rect" + std::to_string(m_rectCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);


		real inner_rounding = glm::min(m_rect_b, m_rect_h) - m_rect_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(rectID, shape, real(m_chunks_x) * real(CHUNK_WIDTH), real(m_chunks_y) * real(CHUNK_WIDTH), inner_rounding, m_rect_rounding, m_particleSpacing, m_mpParameters, m_comodel, sdf::SDF_OPTION::HOLLOW, false, m_fixedPointCloud, m_initVelocity, color);

		t2 = high_resolution_clock::now();



		std::cout << "Finished generating " << pointCloud->N << " points for '" << rectID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_createIsoTriState && m_leftButtonDown)
	{
		m_createIsoTriState = false;

		std::cout << "Mouse position is at (" << m_mouseMpmRenderScreenGridSpace.x << ", " << m_mouseMpmRenderScreenGridSpace.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_isoTriCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::IsoscelesTriangle shape(m_mouseMpmRenderScreenGridSpace, m_iso_tri_b, m_iso_tri_h);
		std::string isoTriID = "isoTri" + std::to_string(m_isoTriCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

		real inner_rounding = glm::min(m_iso_tri_b, m_iso_tri_h) - m_iso_tri_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(isoTriID, shape, real(m_chunks_x) * real(CHUNK_WIDTH), real(m_chunks_y) * real(CHUNK_WIDTH), inner_rounding, m_iso_tri_rounding, m_particleSpacing, m_mpParameters, m_comodel, sdf::SDF_OPTION::HOLLOW, false, m_fixedPointCloud, m_initVelocity, color);

		t2 = high_resolution_clock::now();


		std::cout << "Finished generating " << pointCloud->N << " points for '" << isoTriID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (m_paused && m_selectNodeState && m_leftButtonDown) {
		m_selectNodeState = false;

		m_node[0] = glm::clamp((int)(m_mouseMpmRenderScreenGridSpace.x), 0, GRID_SIZE_X - 1);
		m_node[1] = glm::clamp((int)(m_mouseMpmRenderScreenGridSpace.y), 0, GRID_SIZE_Y - 1);
	}

	if (m_paused && m_addPolygonVertexState && m_leftButtonDown) {
		m_addPolygonVertexState = false;

		real vertexX = m_mouseMpmRenderScreenGridSpace.x;
		real vertexY = m_mouseMpmRenderScreenGridSpace.y;

		vec2 v = vec2(vertexX, vertexY);

		//if (m_mouseMoved) {
		m_polygon->AddVertex(v);
		//}
	}

	if (m_paused && m_addPWLineVertexState && m_leftButtonDown) {
		m_addPWLineVertexState = false;

		real vertexX = m_mouseMpmRenderScreenGridSpace.x;
		real vertexY = m_mouseMpmRenderScreenGridSpace.y;

		vec2 v = vec2(vertexX, vertexY);

		m_pwLine->AddVertex(v);
	}
}