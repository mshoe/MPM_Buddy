#include "MpmGeometryEngine.h"



void mpm::MpmGeometryEngine::GenPointCloudPolygon(std::shared_ptr<sdf::Polygon> polygon, vec2 center)
{
	if (!m_mpmAlgorithmEngine->m_paused)
		return;

	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();

	// First change the center point of the polygon to center:
	vec2 original_center = polygon->center;
	polygon->center = center;




	std::string polygonID = "polygon" + std::to_string(m_mpmEngine->m_polygonCount);
	m_mpmEngine->m_polygonCount++;

	glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

	std::shared_ptr<PointCloud> pointCloud = GenPointCloud(polygonID, *polygon, 
														   real(m_mpmEngine->m_grid->grid_dim_x), real(m_mpmEngine->m_grid->grid_dim_y),
														   0.0, 0.0, m_particleSpacing, 
														   m_mpmAlgorithmEngine->m_mpParameters, m_mpmAlgorithmEngine->m_comodel,
														   sdf::SDF_OPTION::NORMAL, m_invertedSdf, m_fixedPointCloud, 
														   m_initVelocity, color);

	t2 = high_resolution_clock::now();

	// Change the center point of the polygon back to original:
	polygon->center = original_center;

	std::cout << "Finished generating " << pointCloud->N << " points for '" << polygonID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";

}

void mpm::MpmGeometryEngine::GenPointCloudPWLine()
{
	if (!m_mpmAlgorithmEngine->m_paused)
		return;

	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;
	t1 = high_resolution_clock::now();




	std::string pwLineID = "line" + std::to_string(m_mpmEngine->m_pwLineCount);
	m_mpmEngine->m_pwLineCount++;

	glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

	std::shared_ptr<PointCloud> pointCloud = GenPointCloud(pwLineID, *m_pwLine, 
														   real(m_mpmEngine->m_grid->grid_dim_x), real(m_mpmEngine->m_grid->grid_dim_y),
														   0.0, m_pwLineRounding, m_particleSpacing, 
														   m_mpmAlgorithmEngine->m_mpParameters, m_mpmAlgorithmEngine->m_comodel,
														   sdf::SDF_OPTION::ROUNDED, false, m_fixedPointCloud, 
														   m_initVelocity, color);

	t2 = high_resolution_clock::now();

	std::cout << "Finished generating " << pointCloud->N << " points for '" << pwLineID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
}

std::shared_ptr<mpm::PointCloud> mpm::MpmGeometryEngine::GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
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
	pointCloud->parameters.CalculateLameParameters();

	std::cout << "mew: " << pointCloud->parameters.mew << ", lam: " << pointCloud->parameters.lam << std::endl;

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
				mp.SetMaterialParameters(pointCloud->parameters);
				mp.rgba = pointCloud->color;
				// calculate mp.vol in a compute shader (not here)


				pointCloud->points.push_back(mp);
			}
		}
	}
	pointCloud->N = pointCloud->points.size();

	if (pointCloud->N > 0) {
		m_mpmAlgorithmEngine->CalculatePointCloudVolumes(pointCloudID, pointCloud);
		pointCloud->GenPointCloudSSBO();

		

		m_mpmEngine->m_pointCloudMap[pointCloudID] = pointCloud;
	}

	std::cout << "PointCloud size = " << pointCloud->points.size() * sizeof(MaterialPoint) << " bytes." << std::endl;

	return pointCloud;
}

std::shared_ptr<mpm::PointCloud> mpm::MpmGeometryEngine::GenPointCloudFromImage(const std::string pointCloudID, const std::vector<unsigned char>& pixels, 
																				const int image_width, const int image_height, 
																				const real gridDimX, const real gridDimY,
																				const MaterialParameters& parameters, 
																				const ENERGY_MODEL comodel, 
																				vec2 initialVelocity, glm::highp_fvec4 color)
{
	std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();

	pointCloud->color = color;
	pointCloud->parameters = parameters;
	pointCloud->parameters.CalculateLameParameters();

	std::cout << "mew: " << pointCloud->parameters.mew << ", lam: " << pointCloud->parameters.lam << std::endl;

	pointCloud->comodel = comodel;

	pointCloud->fixed = false;

	// preferably image_width == image_height, and gridDimX == gridDimY
	real particle_spacing = gridDimX / real(image_width);

	real mass = particle_spacing * particle_spacing * parameters.density;

	// gen points from sdf
	
	for (int j = 0; j < image_width; j++) {
		for (int i = 0; i < image_height; i++) {

			real x = real(j) * particle_spacing;
			real y = real(image_height - 1 - i) * particle_spacing;

			// have these mixed up for some reason
			glm::vec2 p(x, y);

			int r = int(pixels[4 * i * image_width + 4 * j + 0]);
			int g = int(pixels[4 * i * image_width + 4 * j + 1]);
			int b = int(pixels[4 * i * image_width + 4 * j + 2]);

			if (r == 255 &&
				g == 255 &&
				b == 255) {

				// no material point here
				continue;

			}
			else {
				MaterialPoint mp(p, initialVelocity, GLreal(mass));
				mp.rgba = vec4(double(r)/255.0, double(g)/255.0, double(b)/255.0, 1.0);
				mp.SetMaterialParameters(parameters);
				// calculate mp.vol in a compute shader (not here)
				pointCloud->points.push_back(mp);
			}
		}
	}
	pointCloud->N = pointCloud->points.size();

	if (pointCloud->N > 0) {
		pointCloud->GenPointCloudSSBO();

		m_mpmAlgorithmEngine->CalculatePointCloudVolumes(pointCloudID, pointCloud);

		m_mpmEngine->m_pointCloudMap[pointCloudID] = pointCloud;
	}

	std::cout << "PointCloud size = " << pointCloud->points.size() * sizeof(MaterialPoint) << " bytes." << std::endl;

	return pointCloud;
}

void mpm::MpmGeometryEngine::HandleGeometryStates()
{
	using namespace std::chrono;
	time_point<high_resolution_clock> t1;
	time_point<high_resolution_clock> t2;

	bool paused = m_mpmAlgorithmEngine->m_paused;
	bool rightButtonDown = m_mpmEngine->m_rightButtonDown;
	bool leftButtonDown = m_mpmEngine->m_leftButtonDown;
	vec2 mpmMouse2 = m_mpmEngine->m_mouseMpmRenderScreenGridSpace;
	vec4 mpmMouse4 = m_mpmEngine->m_mouseMpmRenderScreenGridSpaceFull;
	int chunks_x = m_mpmEngine->m_chunks_x;
	int chunks_y = m_mpmEngine->m_chunks_y;

	if (paused && rightButtonDown) {
		ClearCreateStates();
	}

	if (paused && m_createCircleState && leftButtonDown)
	{
		m_createCircleState = false;

		std::cout << "Mouse position is at (" << mpmMouse2.x << ", " << mpmMouse2.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_mpmEngine->m_circleCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Circle shape(mpmMouse2, m_circle_r);
		std::string circleID = "circle" + std::to_string(m_mpmEngine->m_circleCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);
		/*color.x = (float)glm::clamp(m_color[0], 0, 255) / 255.f;
		color.y = (float)glm::clamp(m_color[1], 0, 255) / 255.f;
		color.z = (float)glm::clamp(m_color[2], 0, 255) / 255.f;*/

		real inner_rounding = m_circle_r - m_circle_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(circleID, shape, 
															   real(m_mpmEngine->m_grid->grid_dim_x), real(m_mpmEngine->m_grid->grid_dim_y),
															   inner_rounding, m_circle_rounding, m_particleSpacing, 
															   m_mpmAlgorithmEngine->m_mpParameters, m_mpmAlgorithmEngine->m_comodel,
															   sdf::SDF_OPTION::HOLLOW, false, m_fixedPointCloud, 
															   m_initVelocity, color);

		t2 = high_resolution_clock::now();



		std::cout << "Finished generating " << pointCloud->N << " points for '" << circleID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (paused && m_createRectState && leftButtonDown)
	{
		m_createRectState = false;

		std::cout << "Mouse position is at (" << mpmMouse2.x << ", " << mpmMouse2.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_mpmEngine->m_rectCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::Rectangle shape(mpmMouse2, m_rect_b, m_rect_h);
		std::string rectID = "rect" + std::to_string(m_mpmEngine->m_rectCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);


		real inner_rounding = glm::min(m_rect_b, m_rect_h) - m_rect_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(rectID, shape, 
															   real(m_mpmEngine->m_grid->grid_dim_x), real(m_mpmEngine->m_grid->grid_dim_y),
															   inner_rounding, m_rect_rounding, m_particleSpacing, 
															   m_mpmAlgorithmEngine->m_mpParameters, m_mpmAlgorithmEngine->m_comodel,
															   sdf::SDF_OPTION::HOLLOW, false, m_fixedPointCloud, 
															   m_initVelocity, color);

		t2 = high_resolution_clock::now();



		std::cout << "Finished generating " << pointCloud->N << " points for '" << rectID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	if (paused && m_createIsoTriState && leftButtonDown)
	{
		m_createIsoTriState = false;

		std::cout << "Mouse position is at (" << mpmMouse2.x << ", " << mpmMouse2.y << ")" << std::endl;

		// 1. Create point clouds
		std::cout << "Generating point cloud...\n";

		t1 = high_resolution_clock::now();


		m_mpmEngine->m_isoTriCount++;
		//sdf::sdFunc dCircle(sdf::DemoCircle);
		sdf::IsoscelesTriangle shape(mpmMouse2, m_iso_tri_b, m_iso_tri_h);
		std::string isoTriID = "isoTri" + std::to_string(m_mpmEngine->m_isoTriCount);

		glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

		real inner_rounding = glm::min(m_iso_tri_b, m_iso_tri_h) - m_iso_tri_inner_radius;
		std::shared_ptr<PointCloud> pointCloud = GenPointCloud(isoTriID, shape, 
															   real(m_mpmEngine->m_grid->grid_dim_x), real(m_mpmEngine->m_grid->grid_dim_y),
															   inner_rounding, m_iso_tri_rounding, m_particleSpacing, 
															   m_mpmAlgorithmEngine->m_mpParameters, m_mpmAlgorithmEngine->m_comodel,
															   sdf::SDF_OPTION::HOLLOW, false, m_fixedPointCloud, 
															   m_initVelocity, color);

		t2 = high_resolution_clock::now();


		std::cout << "Finished generating " << pointCloud->N << " points for '" << isoTriID << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";
	}

	// THIS GOES TO mpmEngine
	//if (paused && m_selectNodeState && leftButtonDown) {
	//	m_selectNodeState = false;

	//	m_node[0] = glm::clamp((int)(m_mouseMpmRenderScreenGridSpace.x), 0, GRID_SIZE_X - 1);
	//	m_node[1] = glm::clamp((int)(m_mouseMpmRenderScreenGridSpace.y), 0, GRID_SIZE_Y - 1);
	//}

	if (paused && m_addPolygonVertexState && leftButtonDown) {
		m_addPolygonVertexState = false;


		//if (m_mouseMoved) {
		m_polygon->AddVertex(mpmMouse2);
		//}
	}

	if (paused && m_changePolygonOriginState && leftButtonDown) {
		m_changePolygonOriginState = false;
		m_renderPolygonAtMouseState = false;

		m_polygon->RedefinePolygonOrigin(mpmMouse2);
	}

	if (paused && m_movePolygonState && leftButtonDown) {
		m_movePolygonState = false;
		m_renderPolygonAtMouseState = false;

		m_polygon->center = mpmMouse2;
	}

	if (paused && m_createPolygonState && leftButtonDown) {
		m_createPolygonState = false;
		m_renderPolygonAtMouseState = false;

		GenPointCloudPolygon(m_polygon, mpmMouse2);
	}

	if (paused && m_addPWLineVertexState && leftButtonDown) {
		m_addPWLineVertexState = false;

		m_pwLine->AddVertex(mpmMouse2);
	}
}

void mpm::MpmGeometryEngine::SmallCircle()
{
	m_circle_r = 2;
	m_particleSpacing = 0.25;
}


