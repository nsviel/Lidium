#version 330 core

layout(location = 0) in vec3 in_position;
layout(location = 3) in vec4 in_color;

uniform mat4 MVP;

out vec4 color_vertex;

void main()
{
    //Location
    vec4 XYZ = vec4(in_position, 1.0);
    XYZ =  MVP * XYZ;
    gl_Position = XYZ;

    //Color
    color_vertex = in_color;
}
