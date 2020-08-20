#include "PointCloud.h"

void mpm::MaterialParameters::CalculateLameParameters()
{
	lam = youngMod * poisson / ((1.0 + poisson) * (1.0 - 2.0 * poisson));
	mew = youngMod / (2.0 + 2.0 * poisson);
}

void mpm::MaterialParameters::CalculateYongeAndPoisson()
{
	youngMod = mew * (3.0 * lam + 2.0 * mew) / (lam + mew);
	poisson = lam / (2.0 * (lam + mew));
}


double mpm::MaterialPoint::KE()
{
	return 0.5 * m * glm::dot(v, v);
}

void mpm::MaterialPoint::SetMaterialParameters(const MaterialParameters& parameters)
{
	E = parameters.youngMod;
	poisson = parameters.poisson;

	lam = parameters.lam;
	mew = parameters.mew;
	crit_c = parameters.crit_c;
	crit_s = parameters.crit_s;
	hardening = parameters.hardening;
}

void mpm::MaterialPoint::ImGuiDisplay(bool calcDecomp, bool calcdPdF, bool calcVolumeRatio)
{
	glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
	glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);

	
	ImGui::DisplayNamedGlmMatrixMixColor("B", B, min_color, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("Fe", Fe, min_color, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("Fp", Fp, min_color, max_color);
	ImGui::DisplayNamedGlmMatrixMixColor("P", P, min_color, max_color);

	ImGui::DisplayNamedGlmVecColor("rgba", rgba, min_color);

	ImGui::DisplayNamedGlmVecColor("stress", stress, min_color);
	ImGui::DisplayNamedGlmVecColor("strain", strain, min_color);

	

	ImGui::DisplayNamedGlmVecMixColor("x", x, min_color, max_color);
	ImGui::DisplayNamedGlmVecMixColor("v", v, min_color, max_color);

	ImGui::DisplayNamedGlmRealColor("m", m, max_color);
	ImGui::DisplayNamedGlmRealColor("vol", vol, max_color);
	ImGui::DisplayNamedGlmRealColor("vol0", vol0, max_color);
	ImGui::DisplayNamedGlmRealColor("Lz", Lz, max_color);

	ImGui::DisplayNamedGlmRealColor("E", E, max_color);
	ImGui::DisplayNamedGlmRealColor("poisson", poisson, max_color);
	ImGui::DisplayNamedGlmRealColor("lam", lam, max_color);
	ImGui::DisplayNamedGlmRealColor("mew", mew, max_color);

	ImGui::DisplayNamedGlmRealColor("crit_c", crit_c, max_color);
	ImGui::DisplayNamedGlmRealColor("crit_s", crit_s, max_color);
	ImGui::DisplayNamedGlmRealColor("hardening", hardening, max_color);
	ImGui::DisplayNamedGlmRealColor("padding2", padding2, max_color);

	ImGui::DisplayNamedGlmRealColor("epe", epe, max_color);
	ImGui::DisplayNamedGlmRealColor("selected", selected, max_color);
	ImGui::DisplayNamedGlmRealColor("ke", ke, max_color);
	ImGui::DisplayNamedGlmRealColor("padding4", padding4, max_color);

	if (calcDecomp) {
		mat2 R, S;
		PolarDecomp(Fe, R, S);
		ImGui::DisplayNamedGlmMatrixMixColor("FePolar_R", R, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("FePolar_S", S, min_color, max_color);
		mat2 U, V;
		real sig1, sig2;
		SVD(R, S, U, sig1, sig2, V);
		ImGui::DisplayNamedGlmMatrixMixColor("FeSVD_U", U, min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("FeSVD_S", mat2(sig1, 0.0, 0.0, sig2), min_color, max_color);
		ImGui::DisplayNamedGlmMatrixMixColor("FeSVD_V", V, min_color, max_color);
	}

	if (calcdPdF) {
		mat4 dPdF;
		dPdF = FixedCorotationalElasticity::d2Psi_dF2_Mat4(Fe, lam, mew);
		ImGui::DisplayNamedGlmMatrixMixColor("dPdF try 1", dPdF, min_color, max_color);

		dPdF = FixedCorotationalElasticity::d2Psi_dF2_Mat4_trick(Fe, lam, mew);
		ImGui::DisplayNamedGlmMatrixMixColor("dPdF try 2", dPdF, min_color, max_color);
	}

	if (calcVolumeRatio) {
		ImGui::DisplayNamedGlmRealColor("J", glm::determinant(Fe), max_color);
	}

}

void mpm::MaterialPoint::WriteToFile(std::ofstream& myFile)
{
	myFile << x.x << " " << x.y << std::endl;
	myFile << v.x << " " << v.y << std::endl;
	myFile << m << std::endl;
	myFile << vol << std::endl;


	myFile << lam << std::endl;
	myFile << mew << std::endl;


	// skip lz and padding

	myFile << B[0].x << " " << B[0].y << " " << B[1].x << " " << B[1].y << std::endl;
	myFile << Fe[0].x << " " << Fe[0].y << " " << Fe[1].x << " " << Fe[1].y << std::endl;
	myFile << Fp[0].x << " " << Fp[0].y << " " << Fp[1].x << " " << Fp[1].y << std::endl;
	myFile << P[0].x << " " << P[0].y << " " << P[1].x << " " << P[1].y << std::endl;

	// skip everything else
}

void mpm::MaterialPoint::LoadFromFile(std::ifstream& myFile)
{
	std::string line;
	std::string valueStr;
	std::stringstream ss;

	getline(myFile, line);
	ss.str(line);
	getline(ss, valueStr, ' ');
	x.x = std::stod(valueStr);
	getline(ss, valueStr, ' ');
	x.y = std::stod(valueStr);


	getline(myFile, line);
	ss.clear();
	ss.str(line);
	getline(ss, valueStr, ' ');
	v.x = std::stod(valueStr);
	getline(ss, valueStr, ' ');
	v.y = std::stod(valueStr);

	getline(myFile, line);
	m = std::stod(line);

	getline(myFile, line);
	vol = std::stod(line);

	getline(myFile, line);
	lam = std::stod(line);

	getline(myFile, line);
	mew = std::stod(line);

	getline(myFile, line);
	ss.clear();
	ss.str(line);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			getline(ss, valueStr, ' ');
			B[i][j] = std::stod(valueStr);
		}
	}

	getline(myFile, line);
	ss.clear();
	ss.str(line);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			getline(ss, valueStr, ' ');
			Fe[i][j] = std::stod(valueStr);
		}
	}

	getline(myFile, line);
	ss.clear();
	ss.str(line);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			getline(ss, valueStr, ' ');
			Fp[i][j] = std::stod(valueStr);
		}
	}

	getline(myFile, line);
	ss.clear();
	ss.str(line);
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			getline(ss, valueStr, ' ');
			P[i][j] = std::stod(valueStr);
		}
	}
}

mpm::PointCloud::~PointCloud()
{
	points.clear();
	glDeleteBuffers(1, &ssbo);
};

mpm::PointCloud::PointCloud(const PointCloud& a)
{
	parameters = a.parameters;
	points = a.points;
	comodel = a.comodel;
	GenPointCloudSSBO();
}

mpm::PointCloud::PointCloud(std::shared_ptr<const PointCloud> a)
{
	parameters = a->parameters;
	points = a->points;
	N = points.size();
	color = a->color;
	comodel = a->comodel;
	GenPointCloudSSBO();
}

void mpm::PointCloud::GenPointCloudSSBO()
{
	GLuint pointCloudSSBO;
	glCreateBuffers(1, &pointCloudSSBO);
	ssbo = pointCloudSSBO;
	glNamedBufferStorage(
		ssbo,
		sizeof(MaterialPoint) * points.size(),
		&(points.front().x.x),
		GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // add write bit for cpu mode
	);

	std::cout << sizeof(MaterialPoint) << " bytes per mp" << std::endl;
	std::cout << sizeof(mat2) << " bytes per mat2" << std::endl;
	std::cout << sizeof(vec4) << " bytes per vec4" << std::endl;
	std::cout << sizeof(vec2) << " bytes per vec2" << std::endl;
	std::cout << sizeof(double) << " bytes per double" << std::endl;

	ComputeTotalMass(); // adding this here because every time a point cloud is gen, this function will be called
}

void mpm::PointCloud::SaveToFile(std::string fileName)
{
	//fileName = "..\\SavedMPM\\" + fileName + ".mpm";

	std::ofstream myFile;
	myFile.open(fileName, std::ios::out);
	std::streamsize prevPrec = myFile.precision(16);

	// first write the number of material points

	std::cout << "Writing " << points.size() << " points to file '" << fileName << "'" << std::endl;

	myFile << points.size() << std::endl;
	myFile << color.x << " " << color.y << " " << color.z << " " << color.w << std::endl;
	myFile << size_t(comodel) << std::endl;
	// then write the data itself
	for (size_t i = 0; i < points.size(); i++) {
		points[i].WriteToFile(myFile);
	}
	myFile.precision(prevPrec);

	std::cout << "Finished writing." << std::endl;

}

void mpm::PointCloud::LoadFromFile(std::string fileName)
{
	//fileName = "..\\SavedMPM\\" + fileName + ".mpm";

	std::cout << "Loading point cloud from file " << fileName << std::endl;

	std::string line;
	std::string valueStr;
	std::ifstream myfile(fileName);
	if (myfile.is_open())
	{
		// first is # of pts
		getline(myfile, line);
		N = size_t(std::stoi(line));

		// second is color
		getline(myfile, line);
		std::stringstream ss(line);
		getline(ss, valueStr, ' ');
		color.x = std::stof(valueStr);
		getline(ss, valueStr, ' ');
		color.y = std::stof(valueStr);
		getline(ss, valueStr, ' ');
		color.z = std::stof(valueStr);
		getline(ss, valueStr, ' ');
		color.w = std::stof(valueStr);

		// third is comodel
		getline(myfile, line);
		comodel = ENERGY_MODEL(size_t(std::stoi(line)));

		points.resize(N);

		// now load the point data
		for (size_t i = 0; i < N; i++)
		{
			//cout << line << '\n';
			points[i].LoadFromFile(myfile);
		}
		myfile.close();
	}

	else std::cout << "Unable to open file";

	std::cout << "Finished loading point cloud." << std::endl;

	GenPointCloudSSBO();

}

double mpm::PointCloud::ComputeTotalMass()
{
	totalMass = 0.0;
	for (int i = 0; i < points.size(); i++) {
		totalMass += points[i].m;
	}
	return totalMass;
}

double mpm::PointCloud::SumEPE()
{
	double energy = 0.0;

	// assume energy already calculated

	for (size_t p = 0; p < points.size(); p++) {
		MaterialPoint& mp = points[p];

		energy += mp.epe;
	}

	return energy;
}

double mpm::PointCloud::ComputeMPKE()
{
	double MPKE = 0.0;
	for (size_t p = 0; p < points.size(); p++) {
		MaterialPoint& mp = points[p];

		MPKE += mp.KE();
	}
	return MPKE;
}

double mpm::PointCloud::ComputeGravitionalPotential()
{
	// ASSUME GRAVITY IS POINTING DOWN AND IS 9.81
	double GPE = 0.0;
	for (size_t p = 0; p < points.size(); p++) {
		MaterialPoint& mp = points[p];

		GPE += mp.m * mp.x.y * 9.81;
	}
	return GPE;
}



std::ostream& operator<<(std::ostream& out, const mpm::MaterialPoint& c)
{
	out << std::setprecision(std::numeric_limits<double>::digits10 + 1);
	out << "x: " << glm::to_string(c.x) << "\n";
	out << "v: " << glm::to_string(c.v) << "\n";
	out << "m: " << c.m << "\n";
	out << "vol: " << c.vol << "\n";
	out << "Lz: " << c.Lz << "\n";
	out << "B: " << glm::to_string(c.B) << "\n";
	out << "Fe: " << glm::to_string(c.Fe) << "\n";
	out << "Fe (highp): " << "(" << c.Fe[0][0] << ", " << c.Fe[0][1] << "), (" << c.Fe[1][0] << ", " << c.Fe[1][1] << ")" << "\n";
	out << "Fp: " << glm::to_string(c.Fp) << "\n";
	out << "P: " << glm::to_string(c.P) << "\n";
	out << "epe: " << c.epe << "\n";
	out << "selectedWpg: " << c.selected << "\n";
	return out;
}

void mpm::UpdateStress(MaterialPoint& mp, ENERGY_MODEL comodel)
{
	{
		switch (comodel) {
		case ENERGY_MODEL::LINEAR_ELASTICITY:
			mp.P = LinearElasticity::PKTensor(mp.Fe, mp.lam, mp.mew);
			mp.epe = mp.vol0 * LinearElasticity::EnergyDensity(mp.Fe, mp.lam, mp.mew);
			break;
		case ENERGY_MODEL::NEO_HOOKEAN_ELASTICITY:
			mp.P = NeoHookean::PKTensor(mp.Fe, mp.lam, mp.mew);
			break;
		case ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY:
			mp.P = FixedCorotationalElasticity::PKTensor(mp.Fe, mp.lam, mp.mew);
			mp.epe = mp.vol0 * FixedCorotationalElasticity::EnergyDensity(mp.Fe, mp.lam, mp.mew);
			break;
		case ENERGY_MODEL::SIMPLE_SNOW:
			mp.P = SimpleSnow::PKTensor(mp.Fe, mp.Fp, mp.lam, mp.mew, mp.crit_c, mp.crit_s, mp.hardening);
			break;
		default:
			break;
		}
	}
}
