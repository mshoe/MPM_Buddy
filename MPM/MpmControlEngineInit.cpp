#include "MpmControlEngine.h"

void mpm::MpmControlEngine::InitShaders()
{
	using namespace ShaderPaths;

	// interactive / control shaders
	m_pSetDeformationGradients = std::make_unique<ComputeShader>(
		std::vector<std::string>{interactivePath + "pSetDeformationGradients.comp"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_pMultDeformationGradients = std::make_unique<ComputeShader>(
		std::vector<std::string>{interactivePath + "pMultiplyDeformationGradients.comp"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_pSetLameParamters = std::make_unique<ComputeShader>(
		std::vector<std::string>{interactivePath + "pSetLameParameters.comp"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
}

void mpm::MpmControlEngine::CleanupShaders()
{
}