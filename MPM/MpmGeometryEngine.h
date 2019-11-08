#pragma once

#include "Constants.h"

#include "Shader.h"
#include "Shape.h"
#include "PointCloud.h"
#include "Grid.h"

#include "OpenGLScreen.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <chrono>
#include <functional>
#include <unordered_map>
#include <map>
#include <tuple>

namespace mpm {
	//class MpmGeometryEngine {


	//	std::shared_ptr<StandardShader> m_circleShader = nullptr;
	//	std::shared_ptr<StandardShader> m_polygonShader = nullptr;
	//	std::shared_ptr<StandardShader> m_pwLineShader = nullptr;
	//	std::shared_ptr<StandardShader> m_polygonEditorShader = nullptr;

	//	/******************** GEOMETRY EDITOR ********************/
	//	bool m_fixedPointCloud = false;
	//	bool m_invertedSdf = false;
	//	void HandleGeometryStates();
	//	std::shared_ptr<PointCloud> GenPointCloud(const std::string pointCloudID, sdf::Shape& shape,
	//		const real gridDimX, const real gridDimY,
	//		const real inner_rounding, const real outer_rounding,
	//		const real particle_spacing, const MaterialParameters& parameters,
	//		const ENERGY_MODEL comodel, sdf::SDF_OPTION sdfOption,
	//		bool inverted, bool fixed,
	//		vec2 initialVelocity, glm::highp_fvec4 color);
	//	void ClearCreateStates() {
	//		m_createCircleState = false;
	//		m_createRectState = false;
	//		m_createIsoTriState = false;

	//		m_addPolygonVertexState = false;
	//		m_changePolygonOriginState = false;
	//		m_movePolygonState = false;
	//		m_createPolygonState = false;
	//		m_renderPolygonAtMouseState = false;
	//	}

	//	real m_particleSpacing = 0.25;

	//	std::shared_ptr<sdf::Polygon> m_polygon = nullptr;
	//	std::vector<std::shared_ptr<sdf::Polygon>> m_polygons;
	//	size_t m_polygonSelect = 0;
	//	bool m_addPolygonVertexState = false;
	//	bool m_changePolygonOriginState = false;
	//	bool m_movePolygonState = false;
	//	bool m_createPolygonState = false;
	//	bool m_renderPolygonAtMouseState = false;
	//	void GenPointCloudPolygon(std::shared_ptr<sdf::Polygon> polygon, vec2 center);
	//	int m_polygonCount = 0;
	//	bool m_renderPolygon = false;

	//	// Window just for editing polygons
	//	void InitPolygonEditorScreen();
	//	std::shared_ptr<ImGuiScreen> m_polygonEditorScreen = nullptr;

	//	std::shared_ptr<sdf::PWLine> m_pwLine = nullptr;
	//	bool m_addPWLineVertexState = false;
	//	real m_pwLineRounding = 2.0;
	//	void GenPointCloudPWLine();
	//	int m_pwLineCount = 0;
	//	bool m_renderPWLine = false;
	//	bool m_createCircleState = false;
	//	real m_circle_r = 5.0;
	//	real m_circle_inner_radius = 0.0;
	//	real m_circle_rounding = 0.0;

	//	bool m_createRectState = false;
	//	real m_rect_b = 3.0;
	//	real m_rect_h = 3.0;
	//	real m_rect_inner_radius = 0.0;
	//	real m_rect_rounding = 2.0;

	//	bool m_createIsoTriState = false;
	//	real m_iso_tri_b = 3.0;
	//	real m_iso_tri_h = 3.0;
	//	real m_iso_tri_inner_radius = 0.0;
	//	real m_iso_tri_rounding = 2.0;
	//};
}