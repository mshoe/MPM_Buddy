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

		void CalculateLameParameters();
		void CalculateYongeAndPoisson();
	};

	struct MaterialPoint {
		MaterialPoint() {}
		MaterialPoint(vec2 _x, vec2 _v, GLreal _m) : x(_x), v(_v), m(_m) {}

		// to keep track of data alignment:
		// # of mat2: 4
		// # of vec4: 3
		// # of vec2: 2
		// # of double: 16
		
		


		

		vec2 x = vec2(0.0);
		vec2 v = vec2(0.0);

		double m = 0.0;
		double vol = 0.0;
		double vol0 = 0.0; // initial volume
		double Lz = 0.0; // angular momentum (for RPIC)

		double E = 0.0;
		double poisson = 0.0;
		double lam = 0.0;
		double mew = 0.0;

		double crit_c = 0.0;
		double crit_s = 0.0;
		double hardening = 0.0;
		double padding2 = 4.2;

		double energy = 0.0;
		double selected = 0.0;
		double padding3 = 22;
		double padding4 = 0.89;

		mat2 B = mat2(0.0); // for APIC
		mat2 Fe = mat2(1.0);
		mat2 Fp = mat2(1.0);
		mat2 P = mat2(0.0);

		vec4 rgba = vec4(0.5, 1.0, 0.0, 1.0);

		// FOR MUSL
		vec4 stress = vec4(0.0);
		vec4 strain = vec4(0.0);

		

		void SetMaterialParameters(const MaterialParameters& parameters);

		friend std::ostream& operator << (std::ostream& out, const MaterialPoint& c);

		void ImGuiDisplay(bool calcDecomp, bool calcdPdF, bool calcVolumeRatio);

		void WriteToFile(std::ofstream& myFile);

		void LoadFromFile(std::ifstream& myFile);
	};

	

	// optimize move semantics later
	struct PointCloud {
		PointCloud() {};
		~PointCloud();

		PointCloud(const PointCloud& a);
		PointCloud(std::shared_ptr<const PointCloud> a);

		void GenPointCloudSSBO();

		void SaveToFile(std::string fileName);

		void LoadFromFile(std::string fileName);
		
		double ComputeTotalMass();
		double ComputeCOMKE(); // compute center of mass kinetic energy
		double ComputeMPKE(); // compute total material point kinetic energy
		double ComputeElasticPotential(); // compute elastic potential energy
		double ComputeLinearElasticPotentialMUSL(); 
		double ComputeGravitionalPotential();

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

		double totalMass;
	};
}