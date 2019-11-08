#include "MpmGeometryEngine.h"

void mpm::MpmGeometryEngine::InitShaders()
{
	using namespace ShaderPaths;

	m_circleShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsGeometryPath + "polygon.vs"},
		std::vector<std::string>{graphicsGeometryPath + "circle.gs"},
		std::vector<std::string>{graphicsGeometryPath + "polygon.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_polygonShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsGeometryPath + "polygon.vs"},
		std::vector<std::string>{graphicsGeometryPath + "polygon.gs"},
		std::vector<std::string>{graphicsGeometryPath + "polygon.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_pwLineShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsGeometryPath + "polygon.vs"},
		std::vector<std::string>{graphicsGeometryPath + "pwLine.gs"},
		std::vector<std::string>{graphicsGeometryPath + "polygon.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});

	m_polygonEditorShader = std::make_shared<StandardShader>(
		std::vector<std::string>{graphicsGeometryPath + "polygon.vs"},
		std::vector<std::string>{graphicsGeometryPath + "polygonEditor.gs"},
		std::vector<std::string>{graphicsGeometryPath + "polygon.fs"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});


	m_pLassoTool = std::make_unique<ComputeShader>(
		std::vector<std::string>{interactivePath + "pLassoTool.comp"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});
	m_pClearPointSelection = std::make_unique<ComputeShader>(
		std::vector<std::string>{interactivePath + "pClearPointSelection.comp"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header.comp"});

	m_polygon = std::make_shared<sdf::Polygon>();
	m_pwLine = std::make_shared<sdf::PWLine>();
}

void mpm::MpmGeometryEngine::CleanupShaders()
{
}