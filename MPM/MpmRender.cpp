#include "MpmEngine.h"
#include "imgui/imgui.h"

#include "glm_MATLAB.h"
#include "glm_imgui.h"


void mpm::MpmEngine::ImGuiMpmRenderWindow()
{
	ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar;
	if (ImGui::Begin("MPM Render Window", &m_imguiMpmRenderWindow, windowFlags)) {
		ImGui::SetWindowPos(ImVec2(SRC_WIDTH / 2.0, 0.0));
		ImGui::SetWindowSize(ImVec2(SRC_WIDTH / 2.0, SRC_HEIGHT));
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

		static glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
		static glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);
		ImGui::DisplayNamedBoolColor("mouse moved", m_mouseMoved, max_color, min_color);
	}
	ImGui::End();
}

void mpm::MpmEngine::MpmRender()
{

	m_mpmRenderWindow->BindFBO();
	glViewport(0, 0, (GLsizei)m_mpmRenderWindow->screen_dimensions.x, (GLsizei)m_mpmRenderWindow->screen_dimensions.y);
	glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
	glClear(GL_COLOR_BUFFER_BIT);
	RenderScreenShader(m_zoomPoint, 1.0, m_mpmRenderWindow);
	if (m_viewPointClouds) {
		RenderPointClouds(m_zoomPoint, 1.0, m_mpmRenderWindow, m_pPointCloudShader);
	}
	if (m_viewGrid) {
		RenderGrid(m_zoomPoint, 1.0, m_mpmRenderWindow, m_gridShader);
		if (m_viewGridVector) {
			RenderGrid(m_zoomPoint, 1.0, m_mpmRenderWindow, m_gridShaderVector);
		}
	}
	if (m_viewMarchingSquares) {
		RenderMarchingSquares(m_zoomPoint, 1.0, m_mpmRenderWindow, m_gridShaderMarchingSquares);
	}
	// Render Zoom Border
	if (m_showZoomBorder) {
		m_zoomWindowShader->Use();
		m_zoomWindowShader->SetReal("zoomFactor", m_zoomFactor);
		m_zoomWindowShader->SetVec("zoomPoint", m_zoomPoint);
		m_zoomWindow->RenderLineLoop();
	}

	// Render Shapes
	//if (m_createCircleState) {
	//	RenderCircle(m_zoomPoint, 1.0, m_mpmRenderWindow, m_circleShader);
	//}
	//if (m_renderPolygonAtMouseState) {
	//	RenderPolygon(m_mouseMpmRenderScreenGridSpace, false, m_zoomPoint, 1.0, m_mpmRenderWindow, m_polygonShader);
	//}

	//if (m_renderPolygon) {
	//	RenderPolygon(m_polygon->center, m_addPolygonVertexState, m_zoomPoint, 1.0, m_mpmRenderWindow, m_polygonShader);
	//}

	//if (m_renderPWLine) {
	//	RenderPWLine(m_zoomPoint, 1.0, m_mpmRenderWindow, m_pwLineShader);
	//}

	m_mpmGeometryEngine->Render(m_zoomPoint, 1.0, m_mpmRenderWindow);
	m_mpmRenderWindow->UnbindFBO();

}

void mpm::MpmEngine::ZoomRender()
{
	m_zoomWindow->BindFBO();
	glViewport(0, 0, (GLsizei)m_zoomWindow->screen_dimensions.x, (GLsizei)m_zoomWindow->screen_dimensions.y);
	glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
	glClear(GL_COLOR_BUFFER_BIT);
	//RenderScreenShader(m_zoomPoint, m_zoomFactor, m_zoomWindow);
	if (m_viewPointClouds) {
		RenderPointClouds(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_pPointCloudShader);
	}
	if (m_viewGrid) {
		RenderGrid(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_gridShader);
		if (m_viewGridVector) {
			RenderGrid(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_gridShaderVector);
		}
	}
	// Render polygon

	/*if (m_createCircleState) {
		RenderCircle(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_circleShader);
	}

	if (m_renderPolygonAtMouseState) {
		RenderPolygon(m_mouseMpmRenderScreenGridSpace, false, m_zoomPoint, m_zoomFactor, m_zoomWindow, m_polygonShader);
	}

	if (m_renderPolygon) {
		RenderPolygon(m_polygon->center, m_addPolygonVertexState, m_zoomPoint, m_zoomFactor, m_zoomWindow, m_polygonShader);
	}

	if (m_renderPWLine) {
		RenderPWLine(m_zoomPoint, m_zoomFactor, m_zoomWindow, m_pwLineShader);
	}*/
	m_mpmGeometryEngine->Render(m_zoomPoint, m_zoomFactor, m_zoomWindow);
	m_zoomWindow->UnbindFBO();
}

void mpm::MpmEngine::Render()
{
	glViewport(0, 0, (GLsizei)m_openGLScreen->screen_dimensions.x, (GLsizei)m_openGLScreen->screen_dimensions.y);
	glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
	glClear(GL_COLOR_BUFFER_BIT);

	if (m_imguiMpmRenderWindow)
		MpmRender();
	if (m_imguiZoomWindow)
		ZoomRender();

	//// RENDER MOUSE SHADER
	//RenderScreenShader(m_zoomPoint, 1.0, m_openGLScreen);

	//// Render material point clouds
	//if (m_viewPointClouds) {
	//	RenderPointClouds(m_zoomPoint, 1.0, m_openGLScreen, m_pPointCloudShader);
	//}

	//// Render grid nodes
	//if (m_viewGrid) {
	//	RenderGrid(m_zoomPoint, 1.0, m_openGLScreen, m_gridShader);
	//	if (m_viewGridVector) {
	//		RenderGrid(m_zoomPoint, 1.0, m_openGLScreen, m_gridShaderVector);
	//	}
	//}

	//// Render marching squares
	//if (m_viewMarchingSquares) {
	//	RenderMarchingSquares(m_zoomPoint, 1.0, m_openGLScreen, m_gridShaderMarchingSquares);
	//}

	//// Render Zoom Border
	//if (m_showZoomBorder) {
	//	m_zoomWindowShader->Use();
	//	m_zoomWindowShader->SetVec("iSourceResolution", m_openGLScreen->screen_dimensions);
	//	m_zoomWindowShader->SetVec("iResolution", m_openGLScreen->sim_dimensions);
	//	m_zoomWindowShader->SetVec("iCenter", vec2(m_openGLScreen->center.x, m_openGLScreen->screen_dimensions.y - m_openGLScreen->center.y)); // correct y for glsl
	//	m_zoomWindowShader->SetReal("zoomFactor", m_zoomFactor);
	//	m_zoomWindowShader->SetVec("zoomPoint", m_zoomPoint);

	//	m_zoomWindow->RenderLineLoop();
	//}



	//// Render polygon
	//if (m_renderPolygon) {
	//	RenderPolygon(m_zoomPoint, m_zoomFactor, m_openGLScreen, m_polygonShader);
	//}

	//if (m_renderPWLine) {
	//	RenderPWLine(m_zoomPoint, m_zoomFactor, m_openGLScreen, m_pwLineShader);
	//}

	//// Render the grid border
	//m_borderShader->Use();
	//m_borderShader->SetVec("iCenter", vec2(m_openGLScreen->center.x, m_openGLScreen->screen_dimensions.y - m_openGLScreen->center.y)); // correct y for glsl
	//m_borderShader->SetVec("iResolution", m_openGLScreen->sim_dimensions);
	//m_borderShader->SetVec("iSourceResolution", m_openGLScreen->screen_dimensions);
	//m_borderShader->SetInt("CHUNKS_X", m_chunks_x);
	//m_borderShader->SetInt("CHUNKS_Y", m_chunks_y);
	//glBindVertexArray(VisualizeVAO);
	//glDrawArrays(GL_POINTS, 0, (GLsizei)1);
	//glBindVertexArray(0);

	

}

void mpm::MpmEngine::RenderScreenShader(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen)
{
	m_mouseShader->Use();
	m_mouseShader->SetVec("mouse", m_mouseMpmRenderScreenGridSpaceFull);
	//m_mouseShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	//m_mouseShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	m_mouseShader->SetVec("screenResolution", openGLScreen->screen_dimensions);
	m_mouseShader->SetBool("nodeGraphicsActive", m_nodeGraphicsActive);
	m_mouseShader->SetInt("selectedNodeI", m_node[0]);
	m_mouseShader->SetInt("selectedNodeJ", m_node[1]);
	m_mouseShader->SetReal("zoomFactor", zoomFactor);
	m_mouseShader->SetVec("zoomPoint", zoomPoint);
	openGLScreen->Render();
}

void mpm::MpmEngine::RenderPointClouds(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> pointShader)
{

	// RENDER POINT CLOUD
	pointShader->Use();
	pointShader->SetReal("maxSpeedClamp", m_maxSpeedClamp);
	pointShader->SetReal("minSpeedClamp", m_minSpeedClamp);
	pointShader->SetBool("visualizeSpeed", m_visualizeSpeed);
	pointShader->SetReal("maxEnergyClamp", m_maxEnergyClamp);
	pointShader->SetReal("minEnergyClamp", m_minEnergyClamp);
	pointShader->SetBool("visualizeEnergy", m_visualizeEnergy);
	m_mpmGeometryEngine->SetSelectionUniforms(pointShader);
	pointShader->SetReal("zoomFactor", zoomFactor);
	pointShader->SetVec("zoomPoint", zoomPoint);
	// iResolution and iSourceResolution should be same for the zoom window we make, and iCenter should be the actual center
	//m_pPointCloudShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	//m_pPointCloudShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	//m_pPointCloudShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	//m_pPointCloudShader->SetVec("iCenter", m_openGLScreen->center);
	glBindVertexArray(VisualizeVAO);
	for (std::pair<std::string, std::shared_ptr<PointCloud>> pointCloudPair : m_pointCloudMap) {
		pointShader->SetVec("pointCloudColor", pointCloudPair.second->color);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, pointCloudPair.second->ssbo);
		glDrawArrays(GL_POINTS, 0, (GLsizei)pointCloudPair.second->N);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	}
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderGrid(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader)
{
	gridShader->Use();
	gridShader->SetReal("zoomFactor", zoomFactor);
	gridShader->SetVec("zoomPoint", zoomPoint);
	// iResolution and iSourceResolution should be same for the zoom window we make, and iCenter should be the actual center
	//gridShader->SetVec("iResolution", openGLScreen->sim_dimensions);
	//gridShader->SetVec("iSourceResolution", openGLScreen->screen_dimensions);
	//gridShader->SetVec("iCenter", vec2(openGLScreen->center.x, openGLScreen->screen_dimensions.y - openGLScreen->center.y)); // correct y for glsl
	gridShader->SetBool("viewGridMass", m_viewGridMass);
	gridShader->SetReal("maxNodeMassClamp", m_maxNodeMassClamp);
	gridShader->SetReal("minNodeMassClamp", m_minNodeMassClamp);
	gridShader->SetReal("maxPointSize", m_maxNodeMassPointSize);
	gridShader->SetReal("minNodeMassPointSize", m_minNodeMassPointSize);
	gridShader->SetReal("maxNodeMassPointSize", m_maxNodeMassPointSize);
	gridShader->SetInt("gridPointSizeScalingOption", m_gridPointSizeScalingOption);
	gridShader->SetBool("nodeGraphicsActive", m_nodeGraphicsActive);
	gridShader->SetInt("selectedNodeI", m_node[0]);
	gridShader->SetInt("selectedNodeJ", m_node[1]);
	gridShader->SetReal("dt", m_dt);
	gridShader->SetuInt("selectedVector", unsigned int(m_gridVectorOption));
	gridShader->SetReal("maxGridVectorLength", m_maxGridVectorLength);
	gridShader->SetReal("maxGridVectorVisualLength", m_maxGridVectorVisualLength);
	//gridShader->SetBool("collectiveNodeGraphics", m_collectiveNodeSelectionGraphics);

	glBindVertexArray(VisualizeVAO);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)(GRID_SIZE_X * GRID_SIZE_Y));
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindVertexArray(0);
}

void mpm::MpmEngine::RenderMarchingSquares(vec2 zoomPoint, real zoomFactor, std::shared_ptr<OpenGLScreen> openGLScreen, std::shared_ptr<StandardShader> gridShader)
{
	gridShader->Use();
	gridShader->SetReal("zoomFactor", zoomFactor);
	gridShader->SetVec("zoomPoint", zoomPoint);
	gridShader->SetReal("isoMass", m_isoMass);
	gridShader->SetVec("mscolor", m_marchingSquaresColor);
	gridShader->SetReal("dt", m_dt);

	glBindVertexArray(VisualizeVAO);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, gridSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)(GRID_SIZE_X * GRID_SIZE_Y));
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
	glBindVertexArray(0);
}

