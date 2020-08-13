#include "MpmAlgorithmEngine.h"

void mpm::MpmAlgorithmEngine::InitShaders()
{
	using namespace ShaderPaths;

	// first compile shaders
	m_gReset = std::make_unique<ComputeShader>(
		std::vector<std::string>{computePath + "gResetNodes.comp"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});
	//m_p2gScatter = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{computePath + "p2gScatterParticleAndUpdateNodes.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp", mpmHeadersPath + "shapeFunctions.comp"});
	//m_gUpdate = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{computePath + "gUpdateNodes.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});

	//m_g2pGather = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{computePath + "g2pGatherNodesAndUpdateParticle.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp", mpmHeadersPath + "shapeFunctions.comp", mpmHeadersPath + "energyFunctions.comp"});
	m_p2gCalcVolumes = std::make_unique<ComputeShader>(
		std::vector<std::string>{computePath + "p2gCalculateVolumes.comp"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp", mpmHeadersPath + "shapeFunctions.comp"});
	m_g2pCalcVolumes = std::make_unique<ComputeShader>(
		std::vector<std::string>{computePath + "g2pCalculateVolumes.comp"},
		std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp", mpmHeadersPath + "shapeFunctions.comp"});





	//// implict time integration shaders
	//m_p2g2pDeltaForce = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{implicitPath + "g2p2gDeltaForce.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp", mpmHeadersPath + "shapeFunctions.comp", mpmHeadersPath + "energyFunctions.comp"});
	//m_gConjugateResidualsInitPart1 = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{implicitPath + "gCR_InitPart1.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});
	//m_gConjugateResidualsInitPart2 = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{implicitPath + "gCR_InitPart2.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});
	//m_gConjugateResidualsInitPart3 = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{implicitPath + "gCR_InitPart3.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});

	//m_gConjugateResidualsStepPart1 = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{implicitPath + "gCR_StepPart1.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});
	//m_gConjugateResidualsStepPart2 = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{implicitPath + "gCR_StepPart2.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});
	//m_gConjugateResidualsConclusion = std::make_unique<ComputeShader>(
	//	std::vector<std::string>{implicitPath + "gCR_Conclusion.comp"},
	//	std::vector<std::string>{mpmHeadersPath + "mpm_header1.comp"});

	m_sparseMatrixVis = std::make_unique<StandardShader>(
		std::vector<std::string>{graphicsEigenPath + "sparseMatrix.vs"},
		std::vector<std::string>{},
		std::vector<std::string>{graphicsEigenPath + "sparseMatrix.fs"},
		std::vector<std::string>{});

	InitSparseMatrixWindow();

	// initialize material parameters here for now
	m_energyModels[size_t(ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY)].youngMod = 90000.0;
	m_energyModels[size_t(ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY)].poisson = 0.3;
	m_energyModels[size_t(ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY)].density = 40;

	m_energyModels[size_t(ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY)].youngMod = 90000.0;
	m_energyModels[size_t(ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY)].poisson = 0.3;
	m_energyModels[size_t(ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY)].density = 40;

	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].youngMod = 140000.0;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].poisson = 0.2;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].density = 40;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].crit_c = 0.025;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].crit_s = 0.0075;
	m_energyModels[size_t(ENERGY_MODEL::SIMPLE_SNOW)].hardening = 10.0;

	m_mpParameters = m_energyModels[size_t(ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY)];
}

void mpm::MpmAlgorithmEngine::CleanupShaders()
{
}


void mpm::MpmAlgorithmEngine::InitSparseMatrixWindow()
{
	m_sparseMatrixWindow = std::make_shared<ImGuiScreen>(vec2(450.0, 450.0));
	m_sparseMatrixWindow->center = vec2(225.0, 225.0);
	m_sparseMatrixWindow->sim_dimensions = vec2(450.0, 450.0);
}


