#version 450 core

uniform dvec2 iResolution;
// uniform dmat4 iCamera;
uniform dvec4 iMouse;
// uniform double iTime;

// const dvec3 camera_from = iCamera[3].xyz;
// const dvec3 camera_dir = -iCamera[2].xyz;
// const dvec3 camera_right = iCamera[0].xyz;
// const dvec3 camera_up = iCamera[1].xyz;

const int GRID_SIZE_X = 32*4;
const int GRID_SIZE_Y = 32*4;

uniform bool nodeGraphicsActive;
uniform int nodeI;
uniform int nodeJ;
	
void main() {
	double radius = 0.05;

	// pixel.x is 0 at the middle, -1 at left, +1 at right
	dvec2 norm_pixel = dvec2((gl_FragCoord.x - iResolution.x/2.0)/ iResolution.x * 2.0, gl_FragCoord.y / iResolution.y);


	double dist_to_mouse = length(iMouse.xy - norm_pixel);
	double intensity = iMouse.w * step(-radius, -dist_to_mouse) * (radius - dist_to_mouse) / radius;

	//double intensity = iMouse.w * smoothstep(iMouse.x - radius, norm_pixel.x, iMouse.x + radius) * smoothstep(iMouse.y - radius, norm_pixel.y, iMouse.y + radius);
	vec3 color = vec3(float(intensity), float(intensity), 0.0);
	


	double node_radius = 0.1;
	//vec2 norm_pixel = vec2(gl_FragCoord.x / iResolution.x, 1.0 - gl_FragCoord.y / iResolution.y);
	dvec2 norm_node_pos = dvec2(double(nodeI) / double(GRID_SIZE_X), double(GRID_SIZE_Y - nodeJ) / double(GRID_SIZE_Y));

	double dist_to_node = length(norm_node_pos - norm_pixel);
	double node_intensity = double(nodeGraphicsActive) * step(-node_radius, -dist_to_node) * (node_radius - dist_to_node) / node_radius;

	//vec3 node_color = vec3(0.0, node_intensity, node_intensity);

	color.z = float(node_intensity);

	//if (intensity > 0.0) {
	gl_FragColor = vec4(color, 1.0);
	//}
}