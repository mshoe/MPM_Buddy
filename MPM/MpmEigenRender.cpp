#include "MpmAlgorithmEngine.h"

void mpm::MpmAlgorithmEngine::RenderSparseMatrix(const std::vector<EigenTriplet>& eigenTriplets, std::shared_ptr<ImGuiScreen> imguiScreen, int spMatRows)
{

	imguiScreen->BindFBO();
	glViewport(0, 0, (GLsizei)imguiScreen->screen_dimensions.x, (GLsizei)imguiScreen->screen_dimensions.y);
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT);

	m_sparseMatrixVis->Use();
	m_sparseMatrixVis->SetInt("spMatRows", spMatRows);
	GLuint sparseMatrixSSBO;
	glCreateBuffers(1, &sparseMatrixSSBO);

	GLuint vao = m_mpmEngine->VisualizeVAO;// imguiScreen->VAO;

	glNamedBufferStorage(
		sparseMatrixSSBO,
		sizeof(EigenTriplet) * eigenTriplets.size(),
		&(eigenTriplets.front()),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // adding write bit for debug purposes
	);
	glBindVertexArray(vao);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 9, sparseMatrixSSBO);
	glDrawArrays(GL_POINTS, 0, (GLsizei)eigenTriplets.size());
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 9, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, &sparseMatrixSSBO);
	imguiScreen->UnbindFBO();
}