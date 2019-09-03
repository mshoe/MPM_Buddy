#version 450 core

out vec4 stressColor;
uniform double maxEnergyClamp = 100.0;
uniform double minEnergyClamp = 0.0;

/*** HEADER ***/

void main() {

	// calculate radius
	gl_PointSize = 2.0;

	// only draw on the right half of the window screen
	dvec2 norm_pos = (2.0 * points[gl_VertexID].x - vec2(GRID_SIZE_X, GRID_SIZE_Y))/vec2(GRID_SIZE_X, GRID_SIZE_Y);
	norm_pos.x = norm_pos.x / 2.0;
	norm_pos.x = norm_pos.x + 0.5;

	double energy = points[gl_VertexID].energy;
	vec3 maxColor = vec3(1.0, 0.0, 0.0);
	vec3 midColor = vec3(0.0, 1.0, 0.0);
	vec3 minColor = vec3(0.0, 0.0, 1.0);
	float normalizedEnergy = float(clamp((energy - minEnergyClamp)/(maxEnergyClamp - minEnergyClamp), 0.0, 1.0));
	//float colorMix = float(mix(normalizedEnergy, 0.0, 1.0));

	if (normalizedEnergy > 0.5) {
		normalizedEnergy = 2.0 * normalizedEnergy - 1.0;
		stressColor = vec4(normalizedEnergy * maxColor + (1.0 - normalizedEnergy) * midColor, 1.0);
	} else {
		normalizedEnergy = 2.0 * normalizedEnergy;
		stressColor = vec4(normalizedEnergy * midColor + (1.0 - normalizedEnergy) * minColor, 1.0);
	}

	//stressColor = vec4(colorMix*maxColor + (1.0 - colorMix)*minColor, 1.0);
	

	// gl_Position needs to take float, not double
    gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
}