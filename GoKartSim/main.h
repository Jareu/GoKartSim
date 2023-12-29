#pragma once
#include <glew.h>
#include <glu.h>
#include <SDL.h>
#include <SDL_opengl.h>
#include "imgui.h"

#include "ShaderUtil.h"
#include "Universe.h"
#include "globals.h"

bool initialize();
void initializeValues();
bool initializeSdl();
bool initializeGlew();
bool initializeOpenGl();
bool initShaders();
bool initGeometry();
void initImGui();

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

void renderUi();

bool quit;
inline std::unique_ptr<Universe> universe;
inline std::unique_ptr <ShaderUtil> shaderUtil;
std::unique_ptr<PidData> pid_data;
std::unique_ptr<Controller> test_controller;
std::unique_ptr<ImGuiIO> imgui_io;

//Graphics
int window_width;
int window_height;
SDL_Window* window;
SDL_GLContext gl_context;
ImVec4 clear_color;
GLint resolution_param;
GLuint gRaceDataBuffer;

GLuint         vao, vbo, ebo, tex;
GLuint         vert_shader;
GLuint         frag_shader;
