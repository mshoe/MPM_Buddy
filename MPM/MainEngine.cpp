#include "MainEngine.h"

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void FrameBufferSizeCallback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
	//renderEngine->UpdateResolution(width, height);
}


MainEngine::MainEngine()
{
	Init();
}

MainEngine::~MainEngine()
{
	Cleanup();
}

bool MainEngine::InitShaderPipeline()
{
	m_mainShader = std::make_unique<StandardShader>("mainShader.vs", std::vector<std::string> { "mainShader.fs" });
	return true;
}

bool MainEngine::InitRaymarchingScreen()
{
	float vertices[] = {
		1.f,  1.f, // top right
		1.f, -1.f, // bottom right
		-1.f, -1.f, // bottom left
		-1.f,  1.f, // top left 
	};
	unsigned int indices[] = {  // note that we start from 0!
		0, 1, 3,   // first triangle
		1, 2, 3    // second triangle
	};


	// ..:: Initialization code :: ..
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);
	glCreateVertexArrays(1, &VAO);
	glCreateBuffers(1, &VBO);
	glCreateBuffers(1, &EBO);

	// 1. bind Vertex Array Object
	glBindVertexArray(VAO);

	// 2. copy our vertices array in a vertex buffer for OpenGL to use
	glNamedBufferStorage(VBO, sizeof(vertices), NULL, GL_DYNAMIC_STORAGE_BIT);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glNamedBufferSubData(VBO, 0, sizeof(vertices), vertices);
	//glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	// 3. copy our index array in a element buffer for OpenGL to use
	glNamedBufferStorage(EBO, sizeof(indices), NULL, GL_DYNAMIC_STORAGE_BIT);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glNamedBufferSubData(EBO, 0, sizeof(indices), indices);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	// 4. then set the vertex attributes pointers
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// remember: do NOT unbind the EBO while a VAO is active as the bound element buffer object IS stored in the VAO; keep the EBO bound.
	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	// You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
	// VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
	glBindVertexArray(0);
	return true;
}

bool MainEngine::Init()
{
	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	m_window = glfwCreateWindow(SRC_WIDTH, SRC_HEIGHT, "SDF Ray Marcher", NULL, NULL);
	if (m_window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return false;
	}
	glfwMakeContextCurrent(m_window);
	glfwSetFramebufferSizeCallback(m_window, FrameBufferSizeCallback);

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return false;
	}

	glfwSetWindowShouldClose(m_window, false);

	// Setup Dear ImGui binding
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;

	ImGui_ImplGlfw_InitForOpenGL(m_window, true);
	ImGui_ImplOpenGL3_Init("#version 450");

	// Setup style
	ImGui::StyleColorsDark();

	// Rendering space
	m_camera = std::make_unique<Camera>(glm::vec3(0.f, 0.f, 50.f), glm::vec3(0.f, 0.f, 0.f), glm::vec3(0.f, 1.f, 0.f));

	InitShaderPipeline();
	InitRaymarchingScreen();

	return true;
}

bool MainEngine::Cleanup()
{
	glDeleteVertexArrays(1, &VAO);
	glDeleteBuffers(1, &VBO);
	glDeleteBuffers(1, &EBO);

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwDestroyWindow(m_window);
	glfwTerminate();
	return true;
}

void MainEngine::Loop()
{
	using namespace std::chrono;
	steady_clock::time_point t1 = steady_clock::now();
	float lag = 0.f;

	while (!glfwWindowShouldClose(m_window))
	{
		steady_clock::time_point t2 = steady_clock::now();
		float dt = duration_cast<duration<float>>(t2 - t1).count();
		t1 = t2;
		lag += dt;
		m_time += dt;

		while (lag >= S_PER_UPDATE) {
			ProcessInput(m_window, lag);
			Update(lag);
			lag -= S_PER_UPDATE;
		}

		double xpos, ypos, left_click, right_click;
		glfwGetCursorPos(m_window, &xpos, &ypos);
		left_click = (glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) ? 1.0f : 0.0f;
		right_click = (glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) ? 1.0f : 0.0f;

		m_mainShader->Use();
		m_mainShader->SetMat("iCamera", m_camera->lookat());
		m_mainShader->SetVec("iResolution", glm::vec2(SRC_WIDTH, SRC_HEIGHT));
		m_mainShader->SetVec("iMouse", glm::vec4(xpos, ypos, left_click, right_click));
		m_mainShader->SetFloat("iTime", m_time);

		Render();

		glfwPollEvents();
	}
}

void MainEngine::ProcessInput(GLFWwindow * window, float lag)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}

void MainEngine::Update(float lag)
{
}

void MainEngine::Render()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	
	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);

	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	if (m_cameraImgui) {
		ImGui::Begin("Camera Vectors", &m_cameraImgui);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
		ImGui::Text("Test");
		
		
		if (ImGui::Button("Close Me"))
			m_cameraImgui = false;
		


		ImGui::End();
	}
	// Rendering
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glfwSwapBuffers(m_window);
}

