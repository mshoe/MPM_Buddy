#include "MpmGeometryEngine.h"

void mpm::MpmGeometryEngine::GUI()
{
	if (m_imguiPolygonEditor) ImGuiPolygonEditor();
	if (m_imguiPWLineEditor) ImGuiPWLineEditor();
	if (m_imguiBasicShapesEditor) ImGuiBasicShapesEditor();
	if (m_imguiPointSelector) ImGuiPointSelector();
}

void mpm::MpmGeometryEngine::Menu()
{
	if (ImGui::BeginMenu("Geometry")) {
		if (ImGui::MenuItem("Basic Shapes", "", m_imguiBasicShapesEditor)) {
			m_imguiBasicShapesEditor = !m_imguiBasicShapesEditor;
		}
		if (ImGui::MenuItem("Polygon Editor", "", m_imguiPolygonEditor)) {
			m_imguiPolygonEditor = !m_imguiPolygonEditor;
		}
		if (ImGui::MenuItem("Piecewise Line Editor", "", m_imguiPWLineEditor)) {
			m_imguiPWLineEditor = !m_imguiPWLineEditor;
		}
		if (ImGui::MenuItem("Point Selector", "", m_imguiPointSelector)) {
			m_imguiPointSelector = !m_imguiPointSelector;
		}
		ImGui::EndMenu();
	}
}

void mpm::MpmGeometryEngine::ImGuiBasicShapesEditor()
{
	if (ImGui::Begin("Basic Shapes Editor", &m_imguiBasicShapesEditor)) {



		ImGui::Checkbox("Fixed point cloud", &m_fixedPointCloud);
		ImGui::Checkbox("Inverted SDF (DANGER)", &m_invertedSdf);

		ImGui::InputReal("Point Spacing", &m_particleSpacing, 0.01, 0.1, "%.2f");
		ImGui::Text("");

		ImGui::InputReal("Circle Radius", &m_circle_r, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Circle Inner Radius", &m_circle_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Circle Rounding", &m_circle_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Circle") && m_mpmEngine->m_paused) {
			ClearCreateStates();
			m_createCircleState = true;
		}


		ImGui::InputReal("Rectangle Base Length", &m_rect_b, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Height Length", &m_rect_h, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Inner Radius", &m_rect_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Rounding", &m_rect_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Rectangle") && m_mpmEngine->m_paused) {
			ClearCreateStates();
			m_createRectState = true;
		}

		ImGui::InputReal("Isosceles Triangle Base Length", &m_iso_tri_b, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Height Length", &m_iso_tri_h, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Triangle Inner Radius", &m_iso_tri_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Triangle Rounding", &m_iso_tri_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Triangle") && m_mpmEngine->m_paused) {
			ClearCreateStates();
			m_createIsoTriState = true;
		}

		//ImGui::InputReal("Line m", &m_line_m);
		//ImGui::InputReal("Line b", &m_line_b);
		//ImGui::InputReal("Line2 m", &m_line2_m);
		//ImGui::InputReal("Line2 b", &m_line2_b);
		//if (ImGui::Button("Create below line y = mx + b")) {
		//	GenPointCloudLineDivider();
		//}

		ImGui::Text("");
		
	}
	ImGui::End();
}

void mpm::MpmGeometryEngine::ImGuiPolygonEditor()
{
	static glm::highp_fvec4 min_color(1.0f, 0.0f, 0.0f, 1.0f);
	static glm::highp_fvec4 max_color(0.0f, 1.0f, 0.0f, 1.0f);
	if (ImGui::Begin("Polygon Editor", &m_imguiPolygonEditor)) {

		ImVec2 mousePos = ImGui::GetCursorScreenPos();
		std::string mousePosStr = std::to_string(mousePos.x) + ", " + std::to_string(mousePos.y);
		ImGui::Text(mousePosStr.c_str());

		ImGui::InputReal("Point Spacing", &m_particleSpacing, 0.01, 0.1, "%.2f");
		ImGui::Text("");
		ImGui::Checkbox("Render polygon", &m_renderPolygon);
		if (ImGui::Button("Change Polygon Origin")) {
			m_changePolygonOriginState = true;
		}
		if (ImGui::Button("Move Polygon")) {
			m_movePolygonState = true;
			m_renderPolygonAtMouseState = true;
		}
		if (ImGui::Button("Add Polygon Vertex")) {
			m_addPolygonVertexState = true;
		}
		if (ImGui::Button("Clear Polygon")) {
			m_polygon->vertices.clear();
		}
		if (ImGui::Button("Fill polygon with MPs")) {
			GenPointCloudPolygon(m_polygon, m_polygon->center);
		}
		if (ImGui::Button("Create Polygon")) {
			ClearCreateStates();
			m_createPolygonState = true;
			m_renderPolygonAtMouseState = true;
		}
		ImGui::DisplayNamedGlmVecMixColor("Polygon center", m_polygon->center, min_color, max_color);
		ImGui::DisplayNamedGlmRealColor("Number of vertices", real(m_polygon->vertices.size()), glm::highp_fvec4(1.0));
		ImGui::Text("Polygon vertices:");
		for (int i = 0; i < m_polygon->vertices.size(); i++) {
			ImGui::DisplayGlmVec(m_polygon->vertices[i]);
		}
	}
	ImGui::End();
}

void mpm::MpmGeometryEngine::ImGuiPWLineEditor()
{
	static glm::highp_fvec4 min_color(1.0f, 0.0f, 0.0f, 1.0f);
	static glm::highp_fvec4 max_color(0.0f, 1.0f, 0.0f, 1.0f);
	if (ImGui::Begin("Piecewise Line Editor", &m_imguiPolygonEditor)) {
		ImGui::Checkbox("Render piecewise line", &m_renderPWLine);
		if (ImGui::Button("Add line Vertex")) {
			m_addPWLineVertexState = true;
		}
		if (ImGui::Button("Clear Line")) {
			m_pwLine->vertices.clear();
		}
		if (ImGui::Button("Create PW Line from SDF")) {
			GenPointCloudPWLine();
		}
		ImGui::InputReal("pwLineRounding", &m_pwLineRounding, 0.1, 1.0, "%.6f");
		ImGui::DisplayNamedGlmRealColor("Number of vertices", real(m_pwLine->vertices.size()), glm::highp_fvec4(1.0));
		ImGui::Text("PW Line vertices:");
		for (int i = 0; i < m_pwLine->vertices.size(); i++) {
			ImGui::DisplayGlmVec(m_pwLine->vertices[i]);
		}
	}
	ImGui::End();
}

void mpm::MpmGeometryEngine::ImGuiPointSelector()
{
	if (ImGui::Begin("Point Selector", &m_imguiPointSelector)) {
		ImGui::Checkbox("Visualize selected points", &m_visualizeSelected);
		static float selectColor[4] = { 1.0f, 1.0f, 0.0f, 1.0f };
		selectColor[0] = m_pointSelectColor.x;
		selectColor[1] = m_pointSelectColor.y;
		selectColor[2] = m_pointSelectColor.z;
		selectColor[3] = m_pointSelectColor.w;
		ImGui::ColorEdit4("Selected points color", selectColor);

		if (ImGui::Button("Select points in polygon")) {
			for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_mpmEngine->m_pointCloudMap) {
				SelectPointsInPolygon(pointCloudPair.first);
			}
		}
	}
	ImGui::End();
}
