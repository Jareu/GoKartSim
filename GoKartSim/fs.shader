#version 430 core

in vec2 Texcoord;
out vec4 out_Color;
uniform vec2 resolution;
const float screenSize = min(resolution.x, resolution.y);
const float trackRadius = screenSize * 0.8f * 0.5f; 
const vec2 middle = resolution / 2.f;
const vec4 trackColor = vec4(0.1, 0.1, 0.1, 1.0);
const vec4 white = vec4(1.0, 1.0, 1.0, 1.0);
const float trackThickness = 20.f;
const float startLineThickness = 4.f;
const vec2 startLineUpperLeft = vec2(middle.x + trackRadius - trackThickness, middle.y - startLineThickness);

layout(std430, binding = 0) buffer raceData
{
    float kart_progress[];
}; 

struct Circle {
    vec2 center;
    float radius;
    vec4 color;
};

void drawStartLine() {
    vec2 coord = gl_FragCoord.xy - startLineUpperLeft;
    if (coord.x >= 0.f && coord.y >= 0.f && coord.x <= trackThickness * 2.f && coord.y <= startLineThickness * 2.f)
    {
        int a = int(4.f * (coord.x / trackThickness));
        if (a % 2 == 0 && coord.y < startLineThickness || a % 2 == 1 && coord.y >= startLineThickness) {
            out_Color = white;
        }
    }
}

void drawTrack() {
    float distance = length(middle - gl_FragCoord.xy);
    // Check if the fragment is inside the circle
    if (distance > trackRadius - trackThickness && distance < trackRadius + trackThickness) {
        // Fragment is inside the circle, set color to the circle's color
        out_Color = trackColor;
    }
}

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
    out_Color = vec4(0.0, 0.0, 0.0, 0.0);
    drawTrack();
    drawStartLine();
    for (int i=0; i< kart_progress.length(); i++)
    {
        Circle circle;
        vec2 center = vec2(middle.x + cos(kart_progress[i]) * trackRadius, middle.y + sin(kart_progress[i]) * trackRadius);
        circle.center = center;
        circle.radius = 5;
        circle.color = vec4(1.0, 0.0, 0.0, 1.0); // Red
        drawCircle(gl_FragCoord.xy, circle);
    }
}