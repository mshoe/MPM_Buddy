#version 450 core

out vec4 vs_speedColor;
out vec4 vs_stressColor;
out uint pointID;
uniform double minSpeedClamp = 0.0;
uniform double maxSpeedClamp = 25.0;
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
	
	double energy = points[gl_VertexID].energy;
	vec3 maxColor = vec3(1.0, 0.0, 0.0);
	vec3 midColor = vec3(0.0, 1.0, 0.0);
	vec3 minColor = vec3(0.0, 0.0, 1.0);
	float normalizedEnergy = float(clamp((energy - minEnergyClamp)/(maxEnergyClamp - minEnergyClamp), 0.0, 1.0));

	if (normalizedEnergy > 0.5) {
		normalizedEnergy = 2.0 * normalizedEnergy - 1.0;
		vs_stressColor = vec4(normalizedEnergy * maxColor + (1.0 - normalizedEnergy) * midColor, 1.0);
	} else {
		normalizedEnergy = 2.0 * normalizedEnergy;
		vs_stressColor = vec4(normalizedEnergy * midColor + (1.0 - normalizedEnergy) * minColor, 1.0);
	}

	double speed = length(points[gl_VertexID].v);
	float normalizedSpeed = float(clamp((speed - minSpeedClamp) / (maxSpeedClamp - minSpeedClamp), 0.0, 1.0));
	if (normalizedSpeed > 0.5) {
		normalizedSpeed = 2.0 * normalizedSpeed - 1.0;
		vs_speedColor = vec4(normalizedSpeed * maxColor + (1.0 - normalizedSpeed) * midColor, 1.0);
	} else {
		normalizedSpeed = 2.0 * normalizedSpeed;
		vs_speedColor = vec4(normalizedSpeed * midColor + (1.0 - normalizedSpeed) * minColor, 1.0);
	}
	
	// gl_Position needs to take float, not double
    gl_Position = vec4(float(norm_pos.x), float(norm_pos.y), 0.0, 1.0);
}