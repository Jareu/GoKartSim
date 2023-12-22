#version 430 core
#extension GL_ARB_explicit_attrib_location : enable
#extension GL_NV_uniform_buffer_std430_layout : enable
#define NUM_SIDES 32

layout(points) in;
layout(line_strip, max_vertices = 512) out;
uniform float fAspectRatio;
layout(std430, binding = 0) uniform raceData
{
    float kart_progress[64];
    int num_karts;
};

vec3 color = vec3(1.0, 1.0, 0.0); // Yellow
out vec3 fColor;

const float PI = 3.1415926;
const float radius1 = 0.9;
const float radius2 = 0.05;

const vec2 finish_loc = vec2(radius1, 0.0);
const float finish_w = 0.03;
const float finish_h = 0.01;

void draw_circle(vec2 position, float radius)
{
    // Safe, GLfloats can represent small integers exactly
    for (int i = 0; i <= NUM_SIDES; i++) {

        // Angle between each side in radians
        float ang = PI * 2.0 / NUM_SIDES * i;

        // Offset from center of point
        vec4 offset = vec4(cos(ang) * fAspectRatio * radius, -sin(ang) * radius, 0.0, 0.0);
        gl_Position = gl_in[0].gl_Position + offset + vec4(position, 0.0, 0.0);
        EmitVertex();
    }

    EndPrimitive();
}

void draw_box(vec2 top_left, vec2 bottom_right)
{
    gl_Position = gl_in[0].gl_Position + vec4(top_left, 0.0, 0.0);
    EmitVertex();

    gl_Position = gl_in[0].gl_Position + vec4(bottom_right.x, top_left.y, 0.0, 0.0);
    EmitVertex();

    gl_Position = gl_in[0].gl_Position + vec4(bottom_right, 0.0, 0.0);
    EmitVertex();

    gl_Position = gl_in[0].gl_Position + vec4(top_left.x, bottom_right.y, 0.0, 0.0);
    EmitVertex();

    gl_Position = gl_in[0].gl_Position + vec4(top_left, 0.0, 0.0);
    EmitVertex();

    EndPrimitive();
}

void main()
{
    fColor = color;

    draw_circle(vec2(0.0, 0.0), radius1);

    for (int i = 0; i < num_karts; i++)
    {
        vec2 position = vec2(cos(kart_progress[0]) * fAspectRatio * radius1, sin(kart_progress[0]) * radius1);
        draw_circle(position, radius2);
    }

    draw_box(finish_loc - vec2(finish_w, finish_h), finish_loc + vec2(finish_w, finish_h));
}