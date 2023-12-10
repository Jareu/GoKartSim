#pragma once
#include "ShaderUtil.h"

#include <glew.h>
#include <glu.h>
#include <SDL.h>
#include <SDL_opengl.h>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"


//Starts up SDL, creates window, and initializes OpenGL
bool init();

//Initializes rendering program and clear color
bool initGL();

//Input handler
void handleKeys(unsigned char key, int x, int y);

//Per frame update
void update();

//Renders quad to the screen
void render();

//Frees media and shuts down SDL
void close();

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
ShaderUtil shaderUtil;

//The window we'll be rendering to
SDL_Window* window = nullptr;

//OpenGL context
SDL_GLContext gl_context;

//Render flag
bool gRenderQuad = true;

//Graphics program
GLuint gProgramID = 0;
GLint gVertexPos2DLocation = -1;
GLuint gVBO = 0;
ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.2f, 1.0f);
GLuint gIBO = 0;