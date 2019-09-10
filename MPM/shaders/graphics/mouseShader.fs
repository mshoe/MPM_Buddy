#version 450 core

uniform dvec2 iSourceResolution; // should be vec2(1800, 900)
uniform dvec2 iResolution; // e.g. vec2(900, 900)
uniform dvec2 iCenter; // e.g. vec(900, 450)

uniform dvec4 iMouse;
// uniform double iTime;

// const dvec3 camera_from = iCamera[3].xyz;
// const dvec3 camera_dir = -iCamera[2].xyz;
// const dvec3 camera_right = iCamera[0].xyz;
// const dvec3 camera_up = iCamera[1].xyz;

const int GRID_SIZE_X = 32*4;
const int GRID_SIZE_Y = 32*4;

uniform bool nodeGraphicsActive;
uniform int selectedNodeI;
uniform int selectedNodeJ;

out vec4 fragColor;
	
void main() {
	double radius = 0.05;

	// original borders are x: (-1.0, 1.0), y: (-1.0, 1.0)
	double left_border = (iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double right_border = (iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double bottom_border = (iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0;
	double top_border = (iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0;

	// pixel.x is 0 at the middle, -1 at left, +1 at right
	// gl_FragCoord is mapped from (0, 1) but is in window/screen space.
	dvec2 norm_pixel = dvec2(gl_FragCoord.x/iSourceResolution.x, gl_FragCoord.y / iSourceResolution.y);

	double dist_to_mouse = length(iMouse.xy - norm_pixel);
	double intensity = iMouse.w * step(-radius, -dist_to_mouse) * (radius - dist_to_mouse) / radius;

	//double intensity = iMouse.w * smoothstep(iMouse.x - radius, norm_pixel.x, iMouse.x + radius) * smoothstep(iMouse.y - radius, norm_pixel.y, iMouse.y + radius);
	vec3 color = vec3(float(intensity), float(intensity), 0.0);
	


	double node_radius = 5;
	
	dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);

    // node position in grid space
    dvec2 node_pos = dvec2(selectedNodeI, selectedNodeJ);

	// normalized pixel position in grid space.
	// e.g. (left_border, bottom_border) -> (0, 0), (right_border, top_border) -> (1,1)
	norm_pixel = (2.0 * gl_FragCoord.xy - iSourceResolution) / iSourceResolution;
	dvec2 norm_grid_pixel = (norm_pixel - dvec2(left_border, bottom_border)) / (dvec2(right_border, top_border) - dvec2(left_border, bottom_border));

	// Now map normalized pixel position to actual grid space.
	dvec2 pixel_pos = norm_grid_pixel * grid_vec;

	double dist_to_node = length(pixel_pos - node_pos);
	double node_intensity = double(nodeGraphicsActive) * step(-node_radius, -dist_to_node) * (node_radius - dist_to_node) / node_radius;

	//double node_intensity = (dist_to_node < node_radius) ? 1.0 : 0.0;

	//vec3 node_color = vec3(0.0, node_intensity, node_intensity);

	color.z = float(node_intensity);

	//if (intensity > 0.0) {
	fragColor = vec4(color, 1.0);
	//}
}