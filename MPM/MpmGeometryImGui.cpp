#include "MpmGeometryEngine.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#include "imgui/imfilebrowser.h"
#include <fstream>
#include <sstream>

void mpm::MpmGeometryEngine::GUI()
{
	if (m_imguiPolygonEditor) ImGuiPolygonEditor();
	if (m_imguiPWLineEditor) ImGuiPWLineEditor();
	if (m_imguiBasicShapesEditor) ImGuiBasicShapesEditor();
	if (m_imguiPointSelector) ImGuiPointSelector();
	if (m_imguiImageLoader) ImGuiImageLoader();
	if (m_imguiMesh) ImGuiMesh();
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
		if (ImGui::MenuItem("Image Loader", "", m_imguiImageLoader)) {
			m_imguiImageLoader = !m_imguiImageLoader;
		}
		if (ImGui::MenuItem("Mesher", "", m_imguiMesh)) {
			m_imguiMesh = !m_imguiMesh;
		}
		ImGui::EndMenu();
	}
}

void mpm::MpmGeometryEngine::ImGuiBasicShapesEditor()
{
	if (ImGui::Begin("Basic Shapes Editor", &m_imguiBasicShapesEditor)) {

		ImGui::ColorEdit4("Point Cloud Color", m_color);
		ImGui::InputReal("Initial Velocity X", &m_initVelocity.x, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Initial Velocity Y", &m_initVelocity.y, 0.1, 1.0, "%.1f");

		/*ImGui::Checkbox("Fixed point cloud", &m_fixedPointCloud);
		ImGui::Checkbox("Inverted SDF (DANGER)", &m_invertedSdf);*/

		ImGui::InputReal("Point Spacing", &m_particleSpacing, 0.01, 0.1, "%.2f");
		ImGui::Text("");

		ImGui::InputReal("Circle Radius", &m_circle_r, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Circle Inner Radius", &m_circle_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Circle Rounding", &m_circle_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Circle") && m_mpmAlgorithmEngine->m_paused) {
			ClearCreateStates();
			m_createCircleState = true;
		}


		ImGui::InputReal("Rectangle Base Length", &m_rect_b, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Height Length", &m_rect_h, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Inner Radius", &m_rect_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Rectangle Rounding", &m_rect_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Rectangle") && m_mpmAlgorithmEngine->m_paused) {
			ClearCreateStates();
			m_createRectState = true;
		}

		ImGui::InputReal("Isosceles Triangle Base Length", &m_iso_tri_b, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Height Length", &m_iso_tri_h, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Triangle Inner Radius", &m_iso_tri_inner_radius, 0.1, 1.0, "%.1f");
		ImGui::InputReal("Isosceles Triangle Rounding", &m_iso_tri_rounding, 0.1, 1.0, "%.1f");
		if (ImGui::Button("Create Solid Triangle") && m_mpmAlgorithmEngine->m_paused) {
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
	if (ImGui::Begin("Piecewise Line Editor", &m_imguiPWLineEditor)) {
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

// Simple helper function to load an image into a OpenGL texture with common settings
bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height)
{
	// Load from file
	int image_width = 0;
	int image_height = 0;
	unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);

	if (image_data == NULL)
		return false;

	// Create a OpenGL texture identifier
	GLuint image_texture;
	glGenTextures(1, &image_texture);
	glBindTexture(GL_TEXTURE_2D, image_texture);

	// Setup filtering parameters for display
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Upload pixels into texture
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
	stbi_image_free(image_data);

	*out_texture = image_texture;
	*out_width = image_width;
	*out_height = image_height;

	return true;
}

void mpm::MpmGeometryEngine::ImGuiImageLoader()
{
	if (ImGui::Begin("Image Loader", &m_imguiImageLoader)) {

		static int image_width = 0;
		static int image_height = 0;
		static GLuint image_texture = 0;
		static bool ret = false;

		static ImGui::FileBrowser loadFileDialog;

		static std::vector<unsigned char> pixelData;

		if (ImGui::Button("Open point cloud file")) {
			loadFileDialog.Open();
			loadFileDialog.SetPwd("..\\pngClouds");
		}

		loadFileDialog.Display();

		if (loadFileDialog.HasSelected())
		{
			std::string loadedImagePath = loadFileDialog.GetSelected().string();
			std::cout << "Loading: " << loadedImagePath << std::endl;
			loadFileDialog.ClearSelected();

			GLuint newTexture = 0;

			ret = LoadTextureFromFile(loadedImagePath.c_str(), &newTexture, &image_width, &image_height);

			if (!ret) {
				std::cout << "failed to load image" << std::endl;
				
			}
			else {
				glDeleteTextures(1, &image_texture);
				image_texture = newTexture;
			}
			//unsigned char* image_data = stbi_
		}

		if (ret) {

			ImGui::Text("pointer = %p", image_texture);
			ImGui::Text("size = %d x %d", image_width, image_height);

			static char pointCloudName[50];
			ImGui::InputText("Gen Point Cloud Name", pointCloudName, 50);

			if (ImGui::Button("Gen Point Cloud")) {
				pixelData.resize(4 * image_width * image_height);
				glGetTextureImage(image_texture, 0, GL_RGBA, GL_UNSIGNED_BYTE, 4 * image_width * image_height, pixelData.data());

				
				//for (int j = 0; j < image_height; j++) {
				//	for (int i = 0; i < image_width; i++) {
				//		if (int(pixelData[4 * i * image_height + 4 * j + 0]) == 255 &&
				//			int(pixelData[4 * i * image_height + 4 * j + 1]) == 255 &&
				//			int(pixelData[4 * i * image_height + 4 * j + 2]) == 255) {
				//			std::cout << " ";
				//		}
				//		else {
				//			std::cout << "1";
				//		}

				//		/*std::cout << "( ";
				//		for (int k = 0; k < 4; k++) {
				//			std::cout << int(pixelData[4 * i * image_height + 4 * j + k]) << " ";
				//		}
				//		std::cout << ")";*/

				//	}
				//	std::cout << std::endl;
				//}

				glm::highp_fvec4 color = glm::highp_fvec4(m_color[0], m_color[1], m_color[2], m_color[3]);

				using namespace std::chrono;
				time_point<high_resolution_clock> t1;
				time_point<high_resolution_clock> t2;
				t1 = high_resolution_clock::now();

				std::shared_ptr<PointCloud> pointCloud = GenPointCloudFromImage(std::string(pointCloudName), pixelData, image_width, image_height,
									   real(m_mpmEngine->m_grid->grid_dim_x), real(m_mpmEngine->m_grid->grid_dim_y), 
									   m_mpmAlgorithmEngine->m_mpParameters, m_mpmAlgorithmEngine->m_comodel,
									   m_initVelocity, color);

				
				t2 = high_resolution_clock::now();

				std::cout << "Finished generating " << pointCloud->N << " points for '" << std::string(pointCloudName) << "' point cloud in " << duration_cast<duration<double>>(t2 - t1).count() << " seconds.\n";

				/*std::cout << std::endl << std::endl;
				for (int i = 0; i < pixelData.size(); i++) {
					std::cout << int(pixelData[i]) << " " << std::endl;
				}*/
				//pixel
			}

			ImGui::Image(
				(void*)(intptr_t)image_texture,
				ImVec2((float)image_width, (float)image_height)
			);
		}
	}
	ImGui::End();
}

void mpm::MpmGeometryEngine::ImGuiMesh()
{
	if (ImGui::Begin("Mesher", &m_imguiMesh)) {

		if (ImGui::Button("load mesh file")) {
			/*std::vector<vec2> vertices;
			vertices.push_back(vec2(0.5, 0.5));
			vertices.push_back(vec2(0.5, -0.5));
			vertices.push_back(vec2(-0.5, -0.5));
			vertices.push_back(vec2(-0.5, 0.5));

			std::vector<unsigned int> indices = {
				0, 1, 3,
				1, 2, 3
			};

			m_meshes.push_back(std::make_shared<Mesh>(vertices, indices));*/


			std::shared_ptr<PointCloud> pointCloud = std::make_shared<PointCloud>();
			
			/* from MATLAB example
			E = 1000;% Young's modulus
			nu = 0.3;% Poisson ratio
			rho = 1000;% density
			kappa = 3 - 4 * nu;% Kolosov constant
			mu = E / 2 / (1 + nu); % shear modulus*/


			pointCloud->color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
			pointCloud->parameters = m_mpmAlgorithmEngine->m_mpParameters;
			/*pointCloud->parameters.youngMod = 1000000;
			pointCloud->parameters.poisson = 0.3;
			pointCloud->parameters.density = 1000;*/
			pointCloud->parameters.CalculateLameParameters();

			pointCloud->comodel = m_mpmAlgorithmEngine->m_comodel;

			pointCloud->fixed = false;

			std::ifstream mpmfile;
			mpmfile.open("..\\points.txt", std::ios::in);
			
			std::string line;
			std::string valueStr;
			std::stringstream ss;

			getline(mpmfile, line);
			int numPoints = std::stoi(line);
			std::cout << "numPoints = " << numPoints << std::endl;

			for (int i = 0; i < numPoints; i++) {
				MaterialPoint mp;
				ss.clear();
				mp.rgba = pointCloud->color;

				mp.E = pointCloud->parameters.youngMod;
				mp.poisson = pointCloud->parameters.poisson;
				mp.lam = pointCloud->parameters.lam;
				mp.mew = pointCloud->parameters.mew;

				getline(mpmfile, line);
				ss.str(line);
				getline(ss, valueStr, ' ');
				mp.x.x = std::stod(valueStr);
				getline(ss, valueStr, ' ');
				mp.x.y = std::stod(valueStr);
				pointCloud->points.push_back(mp);

				//std::cout << mp.x.x << ", " << mp.x.y << std::endl;
			}

			for (int i = 0; i < numPoints; i++) {
				MaterialPoint& mp = pointCloud->points[i];
				ss.clear();
				getline(mpmfile, line);
				ss.str(line);
				getline(ss, valueStr, ' ');
				mp.v.x = std::stod(valueStr);
				getline(ss, valueStr, ' ');
				mp.v.y = std::stod(valueStr);
			}

			for (int i = 0; i < numPoints; i++) {
				MaterialPoint& mp = pointCloud->points[i];
				ss.clear();
				getline(mpmfile, line);
				ss.str(line);
				getline(ss, valueStr, ' ');
				mp.vol = std::stod(valueStr);
				mp.vol0 = mp.vol;
				mp.m = mp.vol * pointCloud->parameters.density;
			}

			pointCloud->N = numPoints;
			pointCloud->GenPointCloudSSBO();

			m_mpmEngine->m_pointCloudMap["loadedMesh"] = pointCloud;

			mpmfile.close();
			
		}


		ImGui::DisplayNamedGlmRealColor("number of meshes", (real)m_meshes.size(), glm::highp_fvec4(1.0));
	}
	ImGui::End();
}