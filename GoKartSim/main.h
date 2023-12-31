#pragma once
#include "ShaderUtil.h"
#include "Universe.h"

#include <glew.h>
#include <glu.h>
#include <SDL.h>
#include <SDL_opengl.h>

#include "Controller.h"
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"

bool initialize();
void initializeValues();
bool initializeSdl();
bool initializeGlew();
bool initializeGl();
bool initShaders();
bool initGeometry();

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

int window_width;
int window_height;

SDL_Window* window = nullptr;
SDL_GLContext gl_context;

//Graphics program
ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.15f, 1.0f);
GLint resolution_param;
GLuint gRaceDataBuffer;
bool quit = false;
inline std::unique_ptr<Universe> universe;
inline std::unique_ptr <ShaderUtil> shaderUtil;
std::unique_ptr<Controller> test_controller;

GLuint vao, vbo, ebo;