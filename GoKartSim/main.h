#pragma once
#include "ShaderUtil.h"
#include "Universe.h"

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

ImGuiIO& initImGui();

//Input handler
void handleKeys(unsigned char key, int x, int y);

//Per frame update
void update();

//Renders quad to the screen
void render();

//Frees media and shuts down SDL
void close();

void updateAspectRatio();

void handleEvents();

void renderUi(const ImGuiIO& io);

int window_width = 800;
int window_height = 600;
ShaderUtil shaderUtil;

//The window we'll be rendering to
SDL_Window* window = nullptr;

//OpenGL context
SDL_GLContext gl_context;

//Graphics program
GLuint gProgramID = 0;
GLint gVertexPos2DLocation = -1;
ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.15f, 1.0f);
GLuint gIBO = 0;
GLuint gVAO = 0;
GLuint gRaceDataBuffer = 0;
GLuint ar_param = -1;
bool quit = false;
inline std::unique_ptr<Universe> universe;
