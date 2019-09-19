#version 450 core

uniform dvec2 iSourceResolution; // should be vec2(1800, 900)
uniform dvec2 iResolution; // e.g. vec2(900, 900)
uniform dvec2 iCenter; // e.g. vec(900, 450)

uniform dvec4 iMouse;
// uniform double iTime;

uniform bool nodeGraphicsActive;
uniform int selectedNodeI;
uniform int selectedNodeJ;

uniform dvec2 zoomPoint;
uniform double zoomFactor;

out vec4 fragColor;

/*** HEADER ***/

void main() {

	dvec2 grid_vec = dvec2(GRID_SIZE_X, GRID_SIZE_Y);
	dvec2 norm_zoomPoint = zoomPoint / grid_vec;

	double radius = 0.05;

	// original borders are x: (-1.0, 1.0), y: (-1.0, 1.0)
	double left_border = (iCenter.x - iSourceResolution.x/2.0 - iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double right_border = (iCenter.x - iSourceResolution.x/2.0 + iResolution.x/2.0) / iSourceResolution.x * 2.0;
	double bottom_border = (iCenter.y - iSourceResolution.y/2.0 - iResolution.y/2.0) / iSourceResolution.y * 2.0;
	double top_border = (iCenter.y - iSourceResolution.y/2.0 + iResolution.y/2.0) / iSourceResolution.y * 2.0;

	// map gl_FragCoord to (0, 1) grid space
	dvec2 norm_grid_pixel = (dvec2(gl_FragCoord.x, gl_FragCoord.y) - iCenter + iResolution/2.0) / iResolution;

	// Mouse is (0, 1) covering screen space, but map to (0, 1) in grid space
	// So first map mouse to (-1, 1) in screen space
	// Then map to (0, 1) if within borders actual screen borders (left_border, right_border), (bottom_border, top_border)
	// This also means it is mapped to (0, 1) if within grid space!
	dvec2 norm_grid_mouse = iMouse.xy;
	norm_grid_mouse = 2.0 * norm_grid_mouse - 1.0;
	norm_grid_mouse = (norm_grid_mouse - dvec2(left_border, bottom_border)) / (dvec2(right_border, top_border) - dvec2(left_border, bottom_border));

	double norm_dist_to_mouse = length(norm_grid_mouse - norm_grid_pixel);
	//radius *= zoomFactor;
	double intensity = iMouse.w * step(-radius, -norm_dist_to_mouse) * (radius - norm_dist_to_mouse) / radius;

	//double intensity = iMouse.w * smoothstep(iMouse.x - radius, norm_pixel.x, iMouse.x + radius) * smoothstep(iMouse.y - radius, norm_pixel.y, iMouse.y + radius);
	vec3 color = vec3(float(intensity), float(intensity), 0.0);
	


	// double node_radius = 5;
	


    // node position in grid space
    dvec2 node_pos = dvec2(selectedNodeI, selectedNodeJ);

	// Now map normalized pixel position from normalized grid space to actual grid space.
	dvec2 grid_pixel = norm_grid_pixel * grid_vec;
	dvec2 d = grid_pixel - node_pos;
	double bSpl = BSpline(d.x) * BSpline(d.y);
	double maxBSpl = BSpline(0.0) * BSpline(0.0);
	

	// double dist_to_node = length(pixel_pos - node_pos) / zoomFactor;
	// double node_intensity = double(nodeGraphicsActive) * step(-node_radius, -dist_to_node) * (node_radius - dist_to_node) / node_radius;

	double node_intensity = bSpl / maxBSpl;

	// //vec3 node_color = vec3(0.0, node_intensity, node_intensity);

	color.z = float(node_intensity);

	//if (intensity > 0.0) {
	fragColor = vec4(color, 1.0);
	//}
}