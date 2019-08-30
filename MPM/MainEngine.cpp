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
void APIENTRY
DebugOutput(GLenum source,
	GLenum type,
	GLuint id,
	GLenum severity,
	GLsizei length,
	const GLchar* message,
	const void* userParam) {
	// ignore non-significant error/warning codes
	if (id == 131169 || id == 131185 || id == 131218 || id == 131204) return;

	std::cout << "---------------" << std::endl;
	std::cout << "Debug message (" << id << "): " << message << std::endl;

	switch (source)
	{
	case GL_DEBUG_SOURCE_API:             std::cout << "Source: API"; break;
	case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   std::cout << "Source: Window System"; break;
	case GL_DEBUG_SOURCE_SHADER_COMPILER: std::cout << "Source: Shader Compiler"; break;
	case GL_DEBUG_SOURCE_THIRD_PARTY:     std::cout << "Source: Third Party"; break;
	case GL_DEBUG_SOURCE_APPLICATION:     std::cout << "Source: Application"; break;
	case GL_DEBUG_SOURCE_OTHER:           std::cout << "Source: Other"; break;
	} std::cout << std::endl;

	switch (type)
	{
	case GL_DEBUG_TYPE_ERROR:               std::cout << "Type: Error"; break;
	case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: std::cout << "Type: Deprecated Behaviour"; break;
	case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  std::cout << "Type: Undefined Behaviour"; break;
	case GL_DEBUG_TYPE_PORTABILITY:         std::cout << "Type: Portability"; break;
	case GL_DEBUG_TYPE_PERFORMANCE:         std::cout << "Type: Performance"; break;
	case GL_DEBUG_TYPE_MARKER:              std::cout << "Type: Marker"; break;
	case GL_DEBUG_TYPE_PUSH_GROUP:          std::cout << "Type: Push Group"; break;
	case GL_DEBUG_TYPE_POP_GROUP:           std::cout << "Type: Pop Group"; break;
	case GL_DEBUG_TYPE_OTHER:               std::cout << "Type: Other"; break;
	} std::cout << std::endl;

	switch (severity)
	{
	case GL_DEBUG_SEVERITY_HIGH:         std::cout << "Severity: high"; break;
	case GL_DEBUG_SEVERITY_MEDIUM:       std::cout << "Severity: medium"; break;
	case GL_DEBUG_SEVERITY_LOW:          std::cout << "Severity: low"; break;
	case GL_DEBUG_SEVERITY_NOTIFICATION: std::cout << "Severity: notification"; break;
	} std::cout << std::endl;
	std::cout << std::endl;
}



MainEngine::MainEngine()
{
	Init();
}

MainEngine::~MainEngine()
{
	Cleanup();
}


bool MainEngine::Init()
{
	// glfw: initialize and configure
	// ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	m_window = glfwCreateWindow(SRC_WIDTH, SRC_HEIGHT, "MPM", NULL, NULL);
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


	glEnable(GL_DEBUG_OUTPUT);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
	glDebugMessageCallback(DebugOutput, NULL);
	glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
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
	//m_camera = std::make_unique<Camera>(glm::vec3(0.f, 0.f, 50.f), glm::vec3(0.f, 0.f, 0.f), glm::vec3(0.f, 1.f, 0.f));


	InitShaderPipeline();
	InitScreen();

	m_mpmEngine = std::make_unique<mpm::MpmEngine>();
	m_mpmEngine->InitComputeShaderPipeline();
	m_mouseShader = m_mpmEngine->GetMouseShader();
	//auto fMouseButtonCallback = std::bind(&mpm::MpmEngine::MouseButtonCallback, m_mpmEngine, _1);
	//glfwSetMouseButtonCallback(m_window, fMouseButtonCallback);

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


bool MainEngine::InitShaderPipeline()
{
	//m_mainShader = std::make_unique<StandardShader>(std::vector<std::string>{"mainShader.vs"}, std::vector<std::string> { "mainShader.fs" });
	return true;
}

bool MainEngine::InitScreen()
{
	// Create screen on the right half of window
	// ** VERTICES need to be float in glsl ** //
	float vertices[] = {
		1.f,  1.f, // top right
		1.f, -1.f, // bottom right
		0.f, -1.f, // bottom left
		0.f,  1.f, // top left 
	};
	unsigned int indices[] = {
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

	glBindVertexArray(VAO);

	glNamedBufferStorage(VBO, sizeof(vertices), NULL, GL_DYNAMIC_STORAGE_BIT);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glNamedBufferSubData(VBO, 0, sizeof(vertices), vertices);

	glNamedBufferStorage(EBO, sizeof(indices), NULL, GL_DYNAMIC_STORAGE_BIT);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glNamedBufferSubData(EBO, 0, sizeof(indices), indices);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	return true;
}

bool MainEngine::InitMPM()
{
	return true;
}

void MainEngine::Loop()
{
	using namespace std::chrono;
	steady_clock::time_point t1 = steady_clock::now();
	double lag = 0.0;

	while (!glfwWindowShouldClose(m_window))
	{
		steady_clock::time_point t2 = steady_clock::now();
		real dt = duration_cast<duration<real>>(t2 - t1).count();
		t1 = t2;
		lag += dt;
		m_time += dt;

		while (lag >= S_PER_UPDATE) {
			ProcessInput(m_window, lag);
			Update(lag);
			lag -= S_PER_UPDATE;
		}

		real xpos, ypos, left_click, right_click;
		glfwGetCursorPos(m_window, &xpos, &ypos);
		left_click = (glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) ? 1.0 : 0.0;
		right_click = (glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) ? 1.0 : 0.0;

		
		real mpm_xpos = (xpos - (real)SRC_WIDTH/2.0) / (real) SRC_WIDTH * 2.0;
		// normalize mouse coordinates
		xpos = xpos / (real)SRC_WIDTH;
		ypos = ypos / (real)SRC_HEIGHT;

		// y value is given inverted
		// since mpm engine occurs on right half of screen, xpos needs to be mapped there
		m_mpmEngine->SetMouseValues(vec2(mpm_xpos, 1.0 - ypos), left_click, right_click);
		m_mpmEngine->HandleInput();

		m_mouseShader->Use();
		//m_mouseShader->SetMat("iCamera", m_camera->lookat());
		m_mouseShader->SetVec("iResolution", vec2(SRC_WIDTH, SRC_HEIGHT));
		m_mouseShader->SetVec("iMouse", vec4(mpm_xpos, ypos, left_click, right_click));
		m_mouseShader->SetReal("iTime", m_time);

		Render();

		glfwPollEvents();
	}
}

void MainEngine::ProcessInput(GLFWwindow * window, real lag)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
		m_mpmEngine->SetPausedState(true);

	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
		if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
			m_mpmEngine->SetPausedState(false);
		}
	}

	if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS) {
		m_mpmEngine->SetCreateCircleState(m_mpmEngine->GetPausedState());
	}

	if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {
		m_mpmEngine->SetCreateRectState(m_mpmEngine->GetPausedState());
	}

	if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS) {
		m_mpmEngine->SetCreateIsoTriState(m_mpmEngine->GetPausedState());
	}
}

void MainEngine::Update(real lag)
{
}

void MainEngine::Render()
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	
	
	m_mouseShader->Use();
	m_mouseShader->SetBool("nodeGraphicsActive", m_mpmEngine->m_nodeGraphicsActive);
	m_mouseShader->SetInt("nodeI", m_mpmEngine->m_node[0]);
	m_mouseShader->SetInt("nodeJ", m_mpmEngine->m_node[1]);

	glBindVertexArray(VAO);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);

	m_mpmEngine->Render();
	

	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	if (m_cameraImgui) {
		ImGui::Begin("Runtime Info", &m_cameraImgui);   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		
		


		ImGui::End();
	}
	//ImGui::ShowDemoWindow();
	m_mpmEngine->RenderGUI();
	// Rendering
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glfwSwapBuffers(m_window);
}

