#version 140
#extension GL_ARB_explicit_attrib_location : enable

layout(location = 0) out vec4 frag_colour;

uniform float radius;
uniform vec2 position = vec2(400.0, 400.0);
uniform vec4 color = vec4(1.0, 0.0, 0.0, 0.0);
uniform vec4 borderColor = vec4(0.0, 0.0, 0.0, 0.0);
uniform float borderThickness;

void main()
{
    float distanceX = abs(gl_FragCoord.x - position.x);
    float distanceY = abs(gl_FragCoord.y - position.y);

    if (sqrt(distanceX * distanceX + distanceY * distanceY) > radius)
        discard;
    else if (sqrt(distanceX * distanceX + distanceY * distanceY) <= radius &&
        sqrt(distanceX * distanceX + distanceY * distanceY) >= radius - borderThickness)
        frag_colour = borderColor;
    else
        frag_colour = color;
}