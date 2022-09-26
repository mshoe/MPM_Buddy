#version 450 core

out vec4 fragColor;
uniform int currGridSizeX;
uniform int currGridSizeY;
uniform double maxMass;
uniform double mediumMass;
uniform double minMass;

uniform vec4 maxDensityColor;
uniform vec4 mediumDensityColor;
uniform vec4 minDensityColor;

uniform bool useColorSpectrum;

uniform vec4 backgroundColor;

uniform dvec2 zoomPoint;
uniform double zoomFactor;

uniform dvec2 screenResolution;

uniform bool sharp = false;

uniform bool controlMode = false;

/*** HEADER ***/

double GetNodeMass(in ivec2 gn) {

    if (gn.x < 0 || gn.x >= currGridSizeX || gn.y < 0 || gn.y >= currGridSizeY) {
        return 0.0;
    }

    gridNode node;
    GetNode(gn.x, gn.y, node);
    return node.m;
}

double GetInterpolatedMass(in dvec2 pos) {

    int botLeftNode_i = int(floor(pos.x)) - 1;
	int botLeftNode_j = int(floor(pos.y)) - 1;

    double mass = 0.0;

    for (int i = 0; i <= 3; ++i) {
		for (int j = 0; j <= 3; ++j) {
			int curNode_i = botLeftNode_i + i;
			int curNode_j = botLeftNode_j + j;

			dvec2 dpg = dvec2(curNode_i, curNode_j) - pos;
			double dx = dpg.x;
			double dy = dpg.y;
			double wpg = BSpline(dx) * BSpline(dy);

            mass += wpg * GetNodeMass(ivec2(curNode_j, curNode_i)); // opposite order becuz glsl is column-major but my array is row-major?

        }
    }

    return mass;
    // double massBotLeft = GetNodeMass(ign_botLeft);
    // double massBotRight = GetNodeMass(ign_botLeft + ivec2(1, 0));
    // double massTopLeft = GetNodeMass(ign_botLeft + ivec2(0, 1));
    // double massTopRight = GetNodeMass(ign_botLeft + ivec2(1, 1));

    // double a_y = pos.y - gn_botLeft.y;
    // double a_x = pos.x - gn_botLeft.x;
    // double mixMassLeft = mix(massBotLeft, massTopLeft, a_y);
    // double mixMassRight = mix(massBotRight, massTopRight, a_y);
    

    // return mix(mixMassLeft, mixMassRight, a_x);
}

void main() {

    // given in range (-1, 1)
    //vec2 openGL_norm_pos = gl_FragCoord.xy;

    // (0, 1)
    vec2 ss_pos = gl_FragCoord.xy;

    dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    dvec2 pos = dvec2(double(ss_pos.x) / screenResolution.x * grid_vec.x, double(ss_pos.y) / screenResolution.y * grid_vec.y);

    pos -= zoomPoint;
    pos /= zoomFactor;
    pos += zoomPoint;
    
    if (!controlMode) {
        double temp = pos.x;
        pos.x = pos.y;
        pos.y = temp;
    }

    double mass = GetInterpolatedMass(pos);

    vec4 color;
    if (sharp) {
        
        if (mass >= maxMass) {
            color = maxDensityColor;
        } else if (useColorSpectrum && mass >= mediumMass) {
            color = mediumDensityColor;
        } else if (useColorSpectrum && mass > minMass) {
            color = minDensityColor;
        }
        else {
            color = backgroundColor;
        }

    } else {

        double norm_mass = mass / maxMass;

        
        color.x = mix(backgroundColor.x, maxDensityColor.x, float(norm_mass));
        color.y = mix(backgroundColor.y, maxDensityColor.y, float(norm_mass));
        color.z = mix(backgroundColor.z, maxDensityColor.z, float(norm_mass));
        color.w = mix(backgroundColor.w, maxDensityColor.w, float(norm_mass));
    }

	fragColor = color;
    //fragColor = densityColor;
}