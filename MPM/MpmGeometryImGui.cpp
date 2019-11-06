#include "MpmEngine.h"

void mpm::MpmEngine::ImGuiBasicShapesEditor()
{
	if (ImGui::Begin("Basic Shapes Editor", &m_imguiBasicShapesEditor)) {



		ImGui::Checkbox("Fixed point cloud", &m_fixedPointCloud);
		ImGui::Checkbox("Inverted SDF (DANGER)", &m_invertedSdf);

		ImGui::InputReal("Point Spacing", &m_particleSpacing, 0.01, 0.1, "%.2f");
		ImGui::Text("");

		ImGui::InputReal("Circle Radius", &m_circle_r, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Circle Inner Radius", &m_circle_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Circle Rounding", &m_circle_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Circle") && m_paused) {
			m_createCircleState = true;
		}


		ImGui::InputReal("Rectangle Base Length", &m_rect_b, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Height Length", &m_rect_h, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Inner Radius", &m_rect_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Rounding", &m_rect_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Rectangle") && m_paused) {
			m_createRectState = true;
		}

		ImGui::InputReal("Isosceles Triangle Base Length", &m_iso_tri_b, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Height Length", &m_iso_tri_h, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Triangle Inner Radius", &m_iso_tri_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Triangle Rounding", &m_iso_tri_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Triangle") && m_paused) {
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

void mpm::MpmEngine::ImGuiPolygonEditor()
{
	if (ImGui::Begin("Polygon Editor", &m_imguiPolygonEditor)) {

		ImVec2 mousePos = ImGui::GetCursorScreenPos();
		std::string mousePosStr = std::to_string(mousePos.x) + ", " + std::to_string(mousePos.y);
		ImGui::Text(mousePosStr.c_str());

		ImGui::InputReal("Point Spacing", &m_particleSpacing, 0.01, 0.1, "%.2f");
		ImGui::Text("");
		ImGui::Checkbox("Render polygon", &m_renderPolygon);
		if (ImGui::Button("Add Polygon Vertex")) {
			m_addPolygonVertexState = true;
		}
		if (ImGui::Button("Clear Polygon")) {
			m_polygon->vertices.clear();
		}
		if (ImGui::Button("Fill polygon with MPs")) {
			GenPointCloudPolygon();
		}
		if (ImGui::Button("Select grid nodes")) {

		}
		ImGui::DisplayNamedGlmRealColor("Number of vertices", real(m_polygon->vertices.size()), glm::highp_fvec4(1.0));
		ImGui::Text("Polygon vertices:");
		for (int i = 0; i < m_polygon->vertices.size(); i++) {
			ImGui::DisplayGlmVec(m_polygon->vertices[i]);
		}

		ImGui::Text("");
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
