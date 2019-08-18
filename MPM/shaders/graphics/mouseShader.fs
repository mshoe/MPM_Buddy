#version 450 core

uniform vec2 iResolution;
uniform mat4 iCamera;
uniform vec4 iMouse;
uniform float iTime;

const vec3 camera_from = iCamera[3].xyz;
const vec3 camera_dir = -iCamera[2].xyz;
const vec3 camera_right = iCamera[0].xyz;
const vec3 camera_up = iCamera[1].xyz;


	
void main() {
	float radius = 0.05;
	vec2 norm_pixel = vec2(gl_FragCoord.x / iResolution.x, 1.0 - gl_FragCoord.y / iResolution.y);


	float dist_to_mouse = length(iMouse.xy - norm_pixel);
	float intensity = iMouse.w * step(-radius, -dist_to_mouse) * (radius - dist_to_mouse) / radius;

	//float intensity = iMouse.w * smoothstep(iMouse.x - radius, norm_pixel.x, iMouse.x + radius) * smoothstep(iMouse.y - radius, norm_pixel.y, iMouse.y + radius);
	vec3 color = vec3(intensity, intensity, 0.0);
	
	gl_FragColor = vec4(color, 1.0);
}