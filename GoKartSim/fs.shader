#version 430 core
#extension GL_NV_uniform_buffer_std430_layout : enable

out vec4 fragColor;

layout(std430, binding = 0) uniform raceData
{
    float kart_progress[64];
    int num_karts;
};

struct Circle {
    vec2 center;
    float radius;
    vec4 color;
};

// Function to draw a circle
void drawCircle(vec2 fragCoord, Circle circle) {
    // Calculate the distance from the fragment to the center
    float distance = length(fragCoord - circle.center);

    // Check if the fragment is inside the circle
    if (distance <= circle.radius) {
        // Fragment is inside the circle, set color to the circle's color
        fragColor = circle.color;
    }
    else {
        // Fragment is outside the circle, set color to transparent
        fragColor = vec4(0.0, 0.0, 0.0, 0.0);
    }
}

void main() {
    // Example: Draw two circles with different parameters
    Circle circle1;
    circle1.center = vec2(0.0, 0.0);
    circle1.radius = 0.4;
    circle1.color = vec4(1.0, 0.0, 0.0, 1.0); // Red

    Circle circle2;
    circle2.center = vec2(0.1, 0.3);
    circle2.radius = 0.2;
    circle2.color = vec4(0.0, 1.0, 0.0, 1.0); // Green

    // Call the drawCircle function for each circle
    drawCircle(gl_FragCoord.xy, circle1);
    drawCircle(gl_FragCoord.xy, circle2);

    fragColor = vec4(1.0, 0.0, 0.0, 0.0);
}