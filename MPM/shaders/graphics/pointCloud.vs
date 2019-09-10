#version 450 core

out vec4 vs_stressColor;
out uint pointID;
uniform double maxEnergyClamp = 100.0;
uniform double minEnergyClamp = 0.0;
uniform dvec2 iSourceResolution; // should be vec2(1800, 900)
uniform dvec2 iResolution; // e.g. vec2(900, 900)
uniform dvec2 iCenter; // e.g. vec(900, 450)
uniform double zoomFactor;
uniform dvec2 zoomPoint;
/*** HEADER ***/

void main() {

	// calculate radius
	gl_PointSize = 2.0;
	pointID = gl_VertexID;

	// map position to (-1.0, 1.0)
	dvec2 pos = points[gl_VertexID].x - zoomPoint;
	pos *= zoomFactor;
	pos += zoomPoint;
	dvec2 norm_pos = (2.0*pos - vec2(GRID_SIZE_X, GRID_SIZE_Y))/vec2(GRID_SIZE_X, GRID_SIZE_Y);
	
	// original borders are x: (-1.0, 1.0), y: (-1.0, 1.0)

	double left_border = (iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double right_border = (iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double bottom_border = (iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0;
	double top_border = (iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0;

	// first map norm_pos to (0, 1)
	norm_pos.x = (norm_pos.x + 1.0) / 2.0;
	norm_pos.y = (norm_pos.y + 1.0) / 2.0;

	// then map to the new borders;

	norm_pos.x = mix(left_border, right_border, norm_pos.x);
	norm_pos.y = mix(bottom_border, top_border, norm_pos.y);
	

	//norm_pos.x = norm_pos.x / 2.0;
	//norm_pos.x = norm_pos.x + 0.5;

	double energy = points[gl_VertexID].energy;
	vec3 maxColor = vec3(1.0, 0.0, 0.0);
	vec3 midColor = vec3(0.0, 1.0, 0.0);
	vec3 minColor = vec3(0.0, 0.0, 1.0);
	float normalizedEnergy = float(clamp((energy - minEnergyClamp)/(maxEnergyClamp - minEnergyClamp), 0.0, 1.0));
	//float colorMix = float(mix(normalizedEnergy, 0.0, 1.0));

	if (normalizedEnergy > 0.5) {
		normalizedEnergy = 2.0 * normalizedEnergy - 1.0;
		vs_stressColor = vec4(normalizedEnergy * maxColor + (1.0 - normalizedEnergy) * midColor, 1.0);
	} else {
		normalizedEnergy = 2.0 * normalizedEnergy;
		vs_stressColor = vec4(normalizedEnergy * midColor + (1.0 - normalizedEnergy) * minColor, 1.0);
	}

	//stressColor = vec4(colorMix*maxColor + (1.0 - colorMix)*minColor, 1.0);
	

	// gl_Position needs to take float, not double
    gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
}