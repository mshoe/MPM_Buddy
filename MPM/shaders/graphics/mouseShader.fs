#version 450 core

uniform vec2 iResolution;
uniform mat4 iCamera;
uniform vec4 iMouse;
uniform float iTime;

const vec3 camera_from = iCamera[3].xyz;
const vec3 camera_dir = -iCamera[2].xyz;
const vec3 camera_right = iCamera[0].xyz;
const vec3 camera_up = iCamera[1].xyz;

const int GRID_SIZE_X = 32*4;
const int GRID_SIZE_Y = 32*4;

uniform bool nodeGraphicsActive;
uniform int nodeI;
uniform int nodeJ;
	
void main() {
	float radius = 0.05;
	vec2 norm_pixel = vec2(gl_FragCoord.x / iResolution.x, 1.0 - gl_FragCoord.y / iResolution.y);


	float dist_to_mouse = length(iMouse.xy - norm_pixel);
	float intensity = iMouse.w * step(-radius, -dist_to_mouse) * (radius - dist_to_mouse) / radius;

	//float intensity = iMouse.w * smoothstep(iMouse.x - radius, norm_pixel.x, iMouse.x + radius) * smoothstep(iMouse.y - radius, norm_pixel.y, iMouse.y + radius);
	vec3 color = vec3(intensity, intensity, 0.0);
	


	float node_radius = 0.1;
	//vec2 norm_pixel = vec2(gl_FragCoord.x / iResolution.x, 1.0 - gl_FragCoord.y / iResolution.y);
	vec2 norm_node_pos = vec2(float(nodeI) / float(GRID_SIZE_X), float(GRID_SIZE_Y - nodeJ) / float(GRID_SIZE_Y));

	float dist_to_node = length(norm_node_pos - norm_pixel);
	float node_intensity = float(nodeGraphicsActive) * step(-node_radius, -dist_to_node) * (node_radius - dist_to_node) / node_radius;

	//vec3 node_color = vec3(0.0, node_intensity, node_intensity);

	color.z = node_intensity;

	//if (intensity > 0.0) {
	gl_FragColor = vec4(color, 1.0);
	//}
}