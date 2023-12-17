#version 150 core
#extension GL_ARB_explicit_attrib_location : enable
#define MAX_NUM_KARTS 64
#define NUM_SIDES 64

layout(points) in;
layout(line_strip, max_vertices = 128) out;
uniform float fAspectRatio;
uniform float race_data[MAX_NUM_KARTS];

vec3 color = vec3(1.0, 1.0, 0.0); // Yellow
out vec3 fColor;

const float PI = 3.1415926;

void main()
{
    fColor = color;

    // Safe, GLfloats can represent small integers exactly
    for (int i = 0; i <= NUM_SIDES; i++) {
        // Angle between each side in radians
        float ang = PI * 2.0 / NUM_SIDES * i;

        // Offset from center of point (0.3 to accomodate for aspect ratio)
        vec4 offset = vec4(cos(ang) * fAspectRatio, -sin(ang), 0.0, 0.0);
        gl_Position = gl_in[0].gl_Position + offset;

        EmitVertex();
    }

    EndPrimitive();
}