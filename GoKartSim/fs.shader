#version 430 core

in vec2 Texcoord;
out vec4 out_Color;
uniform sampler2D tex;
uniform float aspect_ratio;
vec2 coord = vec2(Texcoord[0] / aspect_ratio, Texcoord[1]);
const float radius = 200;

layout(std430, binding = 0) buffer raceData
{
    float kart_progress[];
}; 

struct Circle {
    vec2 center;
    float radius;
    vec4 color;
};

void drawCircle(vec2 fragCoord, Circle circle) {
    // Calculate the distance from the fragment to the center
    float distance = length(fragCoord - circle.center);

    // Check if the fragment is inside the circle
    if (distance <= circle.radius) {
        // Fragment is inside the circle, set color to the circle's color
        out_Color = circle.color;
    }
}

void main()
{
    // out_Color = texture(tex, coord);
    out_Color = vec4(0.0 * aspect_ratio, 0.0, 0.0, 0.0);

    for (int i=0; i< kart_progress.length(); i++)
    {
        Circle circle;
        vec2 center = vec2(600 + cos(kart_progress[i]) * radius, 400 + sin(kart_progress[i]) * radius);
        circle.center = center;
        circle.radius = 5;
        circle.color = vec4(1.0, 0.0, 0.0, 1.0); // Red
        drawCircle(gl_FragCoord.xy, circle);
    }
}