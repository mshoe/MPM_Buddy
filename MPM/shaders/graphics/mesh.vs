#version 450 core

layout (location = 0) in vec2 aPos;

void main()
{
    gl_Position = vec4(float(aPos.x), float(aPos.y), 0.0, 1.0);
}