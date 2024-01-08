#include <iostream>﻿
#include <stdio.h>
#include "main.h"
#include "implot.h"
#include "globals.h"
#include "characterisation.h"

void GLAPIENTRY
GlErrorCallback(GLenum source,
    GLenum type,
    GLuint id,
    GLenum severity,
    GLsizei length,
    const GLchar* message,
    const void* userParam)
{
    fprintf(stderr, "GL CALLBACK: %s type = 0x%x, severity = 0x%x, message = %s\n",
        (type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""),
        type, severity, message);
}

bool initializeSdl()
{
    //Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
    {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return false;
    }

    // Use OpenGL 4.3 core
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(
        SDL_GL_CONTEXT_PROFILE_MASK,
        SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    //Create window
    window = SDL_CreateWindow(
        "GoKart Simulator",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        window_width,
        window_height,
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

    if (!window)
    {
        std::cerr << "Window could not be created! SDL Error:" << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    // Initialize rendering context
    gl_context = SDL_GL_CreateContext(window);
    if (!gl_context)
    {
        std::cerr << "OpenGL context could not be created! SDL Error:" << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return false;
    }

    //Use Vsync
    if (SDL_GL_SetSwapInterval(1) < 0)
    {
        std::cerr << "Warning: Unable to set VSync! SDL Error:" << SDL_GetError() << std::endl;
    }

    SDL_GL_MakeCurrent(window, gl_context);
    return true;
}

bool initializeGlew()
{
    glewExperimental = GL_TRUE;
    GLenum glewError = glewInit();

    if (glewError != GLEW_OK)
    {
        std::cerr << "Error initializing GLEW: " << glewGetErrorString(glewError) << std::endl;
        SDL_GL_DeleteContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();

        return false;
    }

    return true;
}

void initializeValues()
{
    test_controller = std::make_unique<Controller>();
    window_width = 800;
    window_height = 600;
    window = nullptr;
    clear_color = ImVec4(0.0f, 0.0f, 0.15f, 1.0f);
    resolution_param = -1;
    gRaceDataBuffer = 0;
    quit = false;
}

bool initialize()
{
    initializeValues();

    if (!initializeSdl())
    {
        std::cerr << "Unable to initialize SDL!" << std::endl;
        return false;
    }

    if (!initializeGlew())
    {
        std::cerr << "Unable to initialize GLEW!" << std::endl;
        return false;
    }

    if (!initializeGl())
    {
        std::cerr << "Unable to initialize OpenGL!" << std::endl;
        return false;
    }

    return true;
}

bool initializeGl()
{
    shaderUtil = std::make_unique<ShaderUtil>();

    if (!initShaders())
    {
        std::cerr << "Unable to initialize shaders" << std::endl;
        return false;
    }

    if (!initGeometry())
    {
        std::cerr << "Unable to initialize geometry" << std::endl;
        return false;
    }

    // glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(GlErrorCallback, 0);

    // Clear color buffer
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);

    return true;
}

bool initShaders()
{
    // load shaders
    try {
        shaderUtil->load("vs.shader", "fs.shader", "");
    }
    catch (std::exception e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    resolution_param = shaderUtil->getUniformLocation("resolution");

    updateAspectRatio();

    return true;
}

bool initGeometry()
{
    const GLfloat verts[4][2] = {
        //  x      y
        { -1.0f,  1.0f }, // TL
        {  1.0f,  1.0f }, // TR
        {  1.0f, -1.0f }, // BR
        { -1.0f, -1.0f } // BL
    };

    const GLint indicies[] = {
        0, 1, 2, 0, 2, 3
    };

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Populate vertex buffer
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);

    // Populate element buffer
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indicies), indicies, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 2, 0);

    return true;
}

ImGuiIO& initImGui()
{
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;

    // Setup Platform/Renderer bindings
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 150");
    return io;
}

void handleKeys(unsigned char key, int x, int y)
{
    //Toggle quad
    if (key == 'q')
    {
        // Do Q things
    }
}

void update()
{
    std::vector<float> gl_race_data{};

    //No per frame update needed
    universe->tick();

    auto race_data = universe->getRaceData();
    int num_karts = race_data.size();

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, gRaceDataBuffer); // bind buffer
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(GLfloat) * num_karts, race_data.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, gRaceDataBuffer);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

void render()
{
    glClear(GL_COLOR_BUFFER_BIT);

    //Bind program
    shaderUtil->useProgram();

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, NULL);

    //Unbind program
    shaderUtil->stopProgram();
}

void updateAspectRatio()
{
    glViewport(0, 0, window_width, window_height);

    shaderUtil->useProgram();

    if (resolution_param != -1)
    {
        glUniform2f(resolution_param, static_cast<float>(window_width), static_cast<float>(window_height));
    }
    else {
        std::cerr << "Couldn't get resolution! " << std::endl;
    }

    shaderUtil->stopProgram();
}

void handleEvents()
{
    SDL_Event event;

    while (SDL_PollEvent(&event) != 0)
    {
        ImGui_ImplSDL2_ProcessEvent(&event);

        //User requests quit
        switch (event.type)
        {
        case SDL_QUIT:
            quit = true;
            break;
        case SDL_TEXTINPUT:
        {
            int x = 0, y = 0;
            SDL_GetMouseState(&x, &y);
            handleKeys(event.text.text[0], x, y);
            break;
        }
        case SDL_WINDOWEVENT:
        {
            if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                window_width = event.window.data1;
                window_height = event.window.data2;
                updateAspectRatio();
            }
            break;
        }
        default:
            break;
        }
    }
}

void renderUi(const ImGuiIO& io)
{
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame(window);
    ImGui::NewFrame();

    // Rendering
    ImGui::Render();
    glViewport(0, 0, static_cast<GLsizei> (io.DisplaySize.x), static_cast<GLsizei> (io.DisplaySize.y));
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_MakeCurrent(window, gl_context);
}

int main(int argc, char* argv[])
{
    if (initialize() == false)
    {
        std::cerr << "Initialization failed!" << std::endl;
        return EXIT_FAILURE;
    }

    auto& io = initImGui();
    universe = std::make_unique<Universe>(SEED);
    // spawn 3 karts with default speed of 1.0
    constexpr double DEFAULT_SPEED = 1.0;

    for (int i = 1; i < 4; i++)
    {
        auto new_kart = universe->spawnGoKart(i);
        new_kart->setTargetSpeed(DEFAULT_SPEED);
        new_kart->placeAtStartLine(i);

        glGenBuffers(1, &gRaceDataBuffer);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, gRaceDataBuffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(GLfloat) * universe->getGoKartCount(), nullptr, GL_DYNAMIC_COPY);

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    // Main loop
    SDL_StartTextInput();

    while (!quit) {
        handleEvents();

        update();

        render();

        renderUi(io);

        SDL_GL_SwapWindow(window);
    }

    close();

    return EXIT_SUCCESS;
}

void close()
{
    SDL_StopTextInput();

    // Cleanup ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glDeleteBuffers(1, &gRaceDataBuffer);

    shaderUtil->cleanUp();

    glDisableVertexAttribArray(0);
    glDeleteBuffers(1, &ebo);
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    window = nullptr;
    SDL_Quit();
}