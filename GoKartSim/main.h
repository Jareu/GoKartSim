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
bool initialize();

//Initializes rendering program and clear color
bool initializeGl();
bool initShaders();
bool initGeometry();
bool initTextures();

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

SDL_Window* window = nullptr;
SDL_GLContext gl_context;

//Graphics program
ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.15f, 1.0f);
GLint ar_param = -1;
GLuint gRaceDataBuffer = 0;
bool quit = false;
inline std::unique_ptr<Universe> universe;
inline std::unique_ptr <ShaderUtil> shaderUtil;

GLuint         vao, vbo, ebo, tex;
GLuint         vert_shader;
GLuint         frag_shader;
