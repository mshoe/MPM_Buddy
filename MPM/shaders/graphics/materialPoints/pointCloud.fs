#version 450 core

in vec4 gs_speedColor;
in vec4 gs_stressColor;
in vec4 gs_pointColor;
in flat int gs_pointSelected;

uniform bool visualizeSelected = false;
uniform vec4 pointSelectColor = vec4(1.0, 1.0, 0.0, 1.0);
uniform bool visualizeSpeed = false;
uniform bool visualizeEnergy = false;
uniform bool individualPointColors = true;
uniform vec4 pointCloudColor = vec4(1.0, 0.0, 0.0, 1.0);

out vec4 fragColor;

void main() {
	//float r = (dot(gl_PointCoord, gl_PointCoord) < 1.0) ? 1.0 : 0.0;
    
    if (visualizeSpeed) {
        fragColor = gs_speedColor;
    }
    else if (visualizeEnergy) {
        fragColor = gs_stressColor;
    }
    else if (individualPointColors) {
        fragColor = gs_pointColor;
    }
    else {
        fragColor = pointCloudColor;
    }

    if (visualizeSelected && (gs_pointSelected != 0)) {
        fragColor = pointSelectColor;
    }
}