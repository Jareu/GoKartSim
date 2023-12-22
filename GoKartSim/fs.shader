#version 430 core

in vec2 Texcoord;
out vec4 out_Color;
uniform sampler2D tex;
uniform float fAspectRatio;

void main()
{
    out_Color = texture(tex, Texcoord);
}