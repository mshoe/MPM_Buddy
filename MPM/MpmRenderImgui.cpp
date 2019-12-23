#include "MpmEngine.h"

void mpm::MpmEngine::ImGuiMpmRenderWindow()
{
	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus;
	if (ImGui::Begin("MPM Render Window", &m_imguiMpmRenderWindow, windowFlags)) {
		ImGui::SetWindowPos(ImVec2(SRC_WIDTH / 2, 20));
		ImGui::SetWindowSize(ImVec2(SRC_WIDTH / 2, SRC_HEIGHT + 5));
		//ImGui::Window
		//ImGui::Window
		//ImGui::GetStyle().WindowRounding = 0.0f;

		ImVec2 mouseGlobalScreen = ImVec2((float)m_mouseGlobalScreen.x, (float)m_mouseGlobalScreen.y);
		std::string mouseGlobalStr = std::to_string(m_mouseGlobalScreen.x) + ", " + std::to_string(m_mouseGlobalScreen.y);


		static ImVec2 windowPos = ImVec2(ImGui::GetCursorScreenPos().x, SRC_HEIGHT - ImGui::GetCursorScreenPos().y);
		//windowPos.y = SRC_HEIGHT - windowPos.y;
		static std::string windowPosStr = std::to_string(windowPos.x) + ", " + std::to_string(windowPos.y);

		static ImVec2 renderScreenBotLeft = ImVec2(windowPos.x, windowPos.y - (float)m_mpmRenderWindow->screen_dimensions.y);
		static std::string rsblStr = std::to_string(renderScreenBotLeft.x) + ", " + std::to_string(renderScreenBotLeft.y);

		ImVec2 mouseInRenderScreen = ImVec2(mouseGlobalScreen.x - renderScreenBotLeft.x, mouseGlobalScreen.y - renderScreenBotLeft.y);
		std::string mouseInRenderScreenStr = std::to_string(mouseInRenderScreen.x) + ", " + std::to_string(mouseInRenderScreen.y);

		m_mouseMpmRenderScreen = vec2(mouseInRenderScreen.x, mouseInRenderScreen.y);
		m_mouseMpmRenderScreenNormalized = m_mouseMpmRenderScreen / m_mpmRenderWindow->screen_dimensions;
		m_mouseMpmRenderScreenGridSpace = vec2(m_mouseMpmRenderScreenNormalized.x * GRID_SIZE_X, m_mouseMpmRenderScreenNormalized.y * GRID_SIZE_Y);
		m_mouseMpmRenderScreenGridSpaceFull = vec4(m_mouseMpmRenderScreenGridSpace, real(int(m_leftButtonDown)), real(int(m_rightButtonDown)));

		//if (!m_paused)
		//	std::cout << m_mouseMpmRenderScreenGridSpaceFull.x << ", " << m_mouseMpmRenderScreenGridSpaceFull.y << ", " << m_mouseMpmRenderScreenGridSpaceFull.z << ", " << m_mouseMpmRenderScreenGridSpaceFull.w << "\n";




		ImGui::Image(
			(void*)(intptr_t)m_mpmRenderWindow->texture,
			ImVec2((float)m_mpmRenderWindow->screen_dimensions.x, (float)m_mpmRenderWindow->screen_dimensions.y),
			ImVec2(0, 1),
			ImVec2(1, 0),
			ImVec4(1, 1, 1, 1),
			ImVec4(1, 1, 1, 1)
		);

		ImGui::Text("global mouse pos: "); ImGui::SameLine(); ImGui::Text(mouseGlobalStr.c_str());
		//ImGui::Text("global window pos: "); ImGui::SameLine(); ImGui::Text(windowPosStr.c_str());
		ImGui::Text("rsbl pos: "); ImGui::SameLine(); ImGui::Text(rsblStr.c_str());
		ImGui::Text("mouse in render screen: "); ImGui::SameLine(); ImGui::Text(mouseInRenderScreenStr.c_str());
		
		std::string mpmMouseStr = glm::to_string(m_mouseMpmRenderScreenGridSpace);
		ImGui::Text("mouse in mpm space: "); ImGui::SameLine(); ImGui::Text(mpmMouseStr.c_str());

		static glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
		static glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);
		ImGui::DisplayNamedBoolColor("mouse moved", m_mouseMoved, max_color, min_color);
	}
	ImGui::End();
}

void mpm::MpmEngine::ImGuiZoomWindow()
{
	static ImGuiWindowFlags windowFlags = 0; 
	if (ImGui::Begin("Zoom Window", &m_imguiZoomWindow, windowFlags)) {
		
		static bool lockWindow = false;

		if (lockWindow) {
			ImGui::SetWindowPos(ImVec2(SRC_WIDTH / 4, 20));
			ImGui::SetWindowSize(ImVec2(m_zoomWindow->screen_dimensions.x + 50, m_zoomWindow->screen_dimensions.y + 200));
		}

		ImGui::Image(
			(void*)(intptr_t)m_zoomWindow->texture,
			ImVec2((float)m_zoomWindow->screen_dimensions.x, (float)m_zoomWindow->screen_dimensions.y),
			ImVec2(0, 1),
			ImVec2(1, 0),
			ImVec4(1, 1, 1, 1),
			ImVec4(1, 1, 1, 1)
		);

		if (ImGui::Button("Spacetime Control Mode")) {
			m_zoomFactor = 4.0;
			m_zoomPoint.x = 0.0;
			m_zoomPoint.y = 0.0;
			lockWindow = true;
			windowFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBringToFrontOnFocus;
		}

		ImGui::InputReal("Zoom Point x: ", &m_zoomPoint.x, 1.0, 10.0, "%.1f");
		ImGui::InputReal("Zoom Point y: ", &m_zoomPoint.y, 1.0, 10.0, "%.1f");
		ImGui::InputReal("Zoom Factor", &m_zoomFactor, 0.5, 2.0, "%.1f");
		ImGui::Checkbox("Show Zoom Border", &m_showZoomBorder);
		ImGui::Checkbox("Move Zoom Window", &m_movingZoomWindow);
		ImGui::Checkbox("Lock window", &lockWindow);
		

		
		//ImVec2 windowPos = ImGui::GetWindowPos(); //ImVec2(ImGui::GetCursorScreenPos().x, ImGui::GetCursorScreenPos().y);
		//std::string windowPosStr = std::to_string(windowPos.x) + ", " + std::to_string(windowPos.y);

		//ImGui::Text("window pos: "); ImGui::SameLine(); ImGui::Text(windowPosStr.c_str());

		//ImVec2 zoomScreenScreenBotLeft = ImVec2(windowPos.x, windowPos.y - (float)m_zoomWindow->screen_dimensions.y);
		//std::string zsblStr = std::to_string(zoomScreenScreenBotLeft.x) + ", " + std::to_string(zoomScreenScreenBotLeft.y);

		//ImVec2 mouseGlobalScreen = ImVec2((float)m_mouseGlobalScreen.x, (float)m_mouseGlobalScreen.y);
		////std::string mouseGlobalStr = std::to_string(m_mouseGlobalScreen.x) + ", " + std::to_string(m_mouseGlobalScreen.y);

		//ImVec2 mouseInZoomScreen = ImVec2(mouseGlobalScreen.x - zoomScreenScreenBotLeft.x, mouseGlobalScreen.y - zoomScreenScreenBotLeft.y);
		//std::string mouseInZoomScreenStr = std::to_string(mouseInZoomScreen.x) + ", " + std::to_string(mouseInZoomScreen.y);

		//ImGui::Text("mouse in zoom screen: "); ImGui::SameLine(); ImGui::Text(mouseInZoomScreenStr.c_str());

		/*ImVec2 mouseInZoomWindowGridSpace = zoomWindowScreenBotLeft;
		std::string mpmMouseStr = glm::to_string(m_mouseMpmRenderScreenGridSpace);
		ImGui::Text("mouse in mpm space: "); ImGui::SameLine(); ImGui::Text(mpmMouseStr.c_str());*/

		
	}
	ImGui::End();
}