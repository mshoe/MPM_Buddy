#pragma once
#include "Structures.h"
#include "Constants.h"
#include "glm_imgui.h"
#include "MpmFunctions.h"
#include "EnergyFunctions.h"

#include <glad/glad.h>
#include <vector>
#include <iomanip>

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
		MaterialPoint(vec2 _x, vec2 _v, GLreal _m) : x(_x), v(_v), m(_m) {}

		vec2 x;
		vec2 v;
		GLreal m;
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