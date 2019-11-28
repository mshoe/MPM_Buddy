#version 450 core

uniform dvec2 screenResolution; // should be vec2(1800, 900)
// uniform dvec2 iResolution; // e.g. vec2(900, 900)
// uniform dvec2 iCenter; // e.g. vec(900, 450)

uniform dvec4 mouse;
// uniform double iTime;

uniform bool nodeGraphicsActive;
uniform int selectedNodeI;
uniform int selectedNodeJ;

uniform dvec2 zoomPoint;
uniform double zoomFactor;

uniform vec4 backgroundColor;

out vec4 fragColor;

/*** HEADER ***/

void main() {

	dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);
	dvec2 norm_zoomPoint = zoomPoint / grid_vec;

	double radius = 5;
	
	dvec2 norm_grid_pixel = dvec2(gl_FragCoord.x, gl_FragCoord.y) / screenResolution;
	dvec2 pixel_grid_space = norm_grid_pixel * grid_vec;
	dvec2 mouse_grid_space = mouse.xy;

	double dist_to_mouse = abs(length(pixel_grid_space - mouse_grid_space));
	double intensity = mouse.w * step(-radius, -dist_to_mouse) * (radius - dist_to_mouse) / radius ;

	float intensity_f = float(intensity);

	vec3 mouseActiveColor = vec3(1.0, 1.0, 0.0);

	float color_x = mix(backgroundColor.x, mouseActiveColor.x, intensity_f);
	float color_y = mix(backgroundColor.y, mouseActiveColor.y, intensity_f);
	float color_z = mix(backgroundColor.z, mouseActiveColor.z, intensity_f);


	vec3 color = vec3(color_x, color_y, color_z);	
	fragColor = vec4(color, 1.0);
}