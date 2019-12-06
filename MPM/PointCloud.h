#pragma once
#include "Structures.h"
#include "Constants.h"
#include "glm_imgui.h"
#include "MpmFunctions.h"
#include "EnergyFunctions.h"

#include <glad/glad.h>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>

namespace mpm {

	enum class ENERGY_MODEL {
		NEO_HOOKEAN_ELASTICITY = 0,
		FIXED_COROTATIONAL_ELASTICITY = 1,
		SIMPLE_SNOW = 2,
		Count
	};

	struct MaterialParameters {

		// Geometry parameters, TODO: move this to something else
		//real particleSpacing = 0.25;
		real density = 0.16;

		// Actual material parameters
		real youngMod = 400.0;
		real poisson = 0.3;

		real lam = 38888.9;
		real mew = 58333.0;

		real crit_c = 0.025;
		real crit_s = 0.0075;
		real hardening = 10.0;

		void CalculateLameParameters() {
			lam = youngMod * poisson / ((1.f + poisson) * (1.f - 2.f * poisson));
			mew = youngMod / (2.f + 2.f * poisson);
		}
	};

	struct MaterialPoint {
		MaterialPoint() {}
		MaterialPoint(vec2 _x, vec2 _v, GLreal _m) : x(_x), v(_v), m(_m) {}

		vec2 x = vec2(0.0);
		vec2 v = vec2(0.0);
		GLreal m = 0.0;
		GLreal vol = 0.0; // initial volume
		real Lz = 0.0; // angular momentum (for RPIC)
		real padding = 1.5; // padding
		mat2 B = mat2(0.0); // for APIC
		mat2 Fe = mat2(1.0);
		mat2 Fp = mat2(1.0);
		mat2 P = mat2(0.0);

		real lam = 0.0;
		real mew = 0.0;
		real crit_c = 0.0;
		real crit_s = 0.0;
		real hardening = 0.0;
		real padding2 = 4.2;

		// extra not neccessary to store, but useful for debugging:
		mat2 FePolar_R = mat2(1.0);
		mat2 FePolar_S = mat2(1.0);
		mat2 FeSVD_U = mat2(1.0);
		mat2 FeSVD_S = mat2(1.0);
		mat2 FeSVD_V = mat2(1.0);
		mat2 A = mat2(0.0);



		real energy = 0.0;
		real selected = 0.0;

		void SetMaterialParameters(const MaterialParameters& parameters) {
			lam = parameters.lam;
			mew = parameters.mew;
			crit_c = parameters.crit_c;
			crit_s = parameters.crit_s;
			hardening = parameters.hardening;
		}

		friend std::ostream& operator << (std::ostream& out, const MaterialPoint& c) {
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
			out << "FePolar_R: " << glm::to_string(c.FePolar_R) << "\n";
			out << "FePolar_S: " << glm::to_string(c.FePolar_S) << "\n";
			out << "FeSVD_U: " << glm::to_string(c.FeSVD_U) << "\n";
			out << "FeSVD_S: " << glm::to_string(c.FeSVD_S) << "\n";
			out << "FeSVD_V: " << glm::to_string(c.FeSVD_V) << "\n";
			out << "energy: " << c.energy << "\n";
			out << "selectedWpg: " << c.selected << "\n";
			return out;
		}

		void ImGuiDisplay(bool calcDecomp, bool calcdPdF, bool calcVolumeRatio) {
			glm::highp_fvec4 min_color = glm::highp_fvec4(1.0, 0.0, 0.0, 1.0);
			glm::highp_fvec4 max_color = glm::highp_fvec4(0.0, 1.0, 0.0, 1.0);

			ImGui::DisplayNamedGlmVecMixColor("x", x, min_color, max_color);
			ImGui::DisplayNamedGlmVecMixColor("v", v, min_color, max_color);
			ImGui::DisplayNamedGlmRealColor("m", m, max_color);
			ImGui::DisplayNamedGlmRealColor("vol", vol, max_color);
			ImGui::DisplayNamedGlmRealColor("Lz", Lz, max_color);
			ImGui::DisplayNamedGlmRealColor("padding", padding, max_color);
			ImGui::DisplayNamedGlmRealColor("lam", lam, max_color);
			ImGui::DisplayNamedGlmRealColor("mew", mew, max_color);
			ImGui::DisplayNamedGlmRealColor("crit_c", crit_c, max_color);
			ImGui::DisplayNamedGlmRealColor("crit_s", crit_s, max_color);
			ImGui::DisplayNamedGlmRealColor("hardening", hardening, max_color);
			ImGui::DisplayNamedGlmRealColor("padding2", padding2, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("B", B, min_color, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("Fe", Fe, min_color, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("Fp", Fp, min_color, max_color);
			ImGui::DisplayNamedGlmMatrixMixColor("P", P, min_color, max_color);

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
			else {
				ImGui::DisplayNamedGlmMatrixMixColor("FePolar_R", FePolar_R, min_color, max_color);
				ImGui::DisplayNamedGlmMatrixMixColor("FePolar_S", FePolar_S, min_color, max_color);
				ImGui::DisplayNamedGlmMatrixMixColor("FeSVD_U", FeSVD_U, min_color, max_color);
				ImGui::DisplayNamedGlmMatrixMixColor("FeSVD_S", FeSVD_S, min_color, max_color);
				ImGui::DisplayNamedGlmMatrixMixColor("FeSVD_V", FeSVD_V, min_color, max_color);
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
			
			ImGui::DisplayNamedGlmRealColor("energy", energy, max_color);
			//ImGui::DisplayNamedBoolColor("padding1", padding1, max_color, min_color);
			ImGui::DisplayNamedGlmRealColor("selected", selected, max_color);
			//ImGui::DisplayNamedBoolColor("padding2", padding2, max_color, min_color);
			//ImGui::DisplayNamedBoolColor("padding3", padding3, max_color, min_color);
		}

		void WriteToFile(std::ofstream& myFile) {
			myFile << x.x << " " << x.y << std::endl;
			myFile << v.x << " " << v.y << std::endl;
			myFile << m << std::endl;
			myFile << vol << std::endl;

			
			myFile << lam << std::endl;
			myFile << mew << std::endl;


			// skip lz and padding

			myFile << B[0].x << " " << B[0].y << " " << B[1].x << " " << B[1].y <<  std::endl;
			myFile << Fe[0].x << " " << Fe[0].y << " " << Fe[1].x << " " << Fe[1].y << std::endl;
			myFile << Fp[0].x << " " << Fp[0].y << " " << Fp[1].x << " " << Fp[1].y << std::endl;
			myFile << P[0].x << " " << P[0].y << " " << P[1].x << " " << P[1].y << std::endl;

			// skip everything else
		}

		void LoadFromFile(std::ifstream& myFile) {
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
	};

	

	// optimize move semantics later
	struct PointCloud {
		PointCloud() {};
		~PointCloud() {
			points.clear();
			glDeleteBuffers(1, &ssbo);
		};

		PointCloud(const PointCloud& a) {
			parameters = a.parameters;
			points = a.points;
			comodel = a.comodel;
			GenPointCloudSSBO();
		}
		PointCloud(std::shared_ptr<const PointCloud> a) {
			parameters = a->parameters;
			points = a->points;
			N = points.size();
			color = a->color;
			comodel = a->comodel;
			GenPointCloudSSBO();
		}

		void GenPointCloudSSBO() {
			GLuint pointCloudSSBO;
			glCreateBuffers(1, &pointCloudSSBO);
			ssbo = pointCloudSSBO;
			glNamedBufferStorage(
				ssbo,
				sizeof(MaterialPoint) * points.size(),
				&(points.front().x.x),
				GL_MAP_READ_BIT | GL_MAP_WRITE_BIT // add write bit for cpu mode
			);
		}

		void SaveToFile(std::string fileName) {
			fileName = "..\\SavedMPM\\" + fileName + ".mpm";

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

		void LoadFromFile(std::string fileName) {
			fileName = "..\\SavedMPM\\" + fileName + ".mpm";

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
				color.x = std::stod(valueStr);
				getline(ss, valueStr, ' ');
				color.y = std::stod(valueStr);
				getline(ss, valueStr, ' ');
				color.z = std::stod(valueStr);
				getline(ss, valueStr, ' ');
				color.w = std::stod(valueStr);

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
		
		

		size_t N = 0;
		std::vector<MaterialPoint> points;
		//std::vector<MaterialPointPhysicalProperties> properties;

		glm::highp_fvec4 color = glm::highp_fvec4(1.f, 0.f, 0.f, 1.f);

		MaterialParameters parameters;

		/*real mew = 0.0;
		real lam = 0.0;*/

		ENERGY_MODEL comodel = ENERGY_MODEL::FIXED_COROTATIONAL_ELASTICITY;

		GLuint ssbo = 0;

		bool fixed = false;
	};
}