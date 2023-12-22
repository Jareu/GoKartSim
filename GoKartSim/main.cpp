#include <stdio.h>
#include <iostream>
#include "main.h"
#include "logo.h"
#include "globals.h"

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

bool initialize()
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

    //Initialize GLEW
    glewExperimental = GL_TRUE;
    GLenum glewError = glewInit();

    if (glewError != GLEW_OK)
    {
        std::cerr << "Error initializing GLEW!" << glewGetErrorString(glewError) << std::endl;
        SDL_GL_DeleteContext(gl_context);
        SDL_DestroyWindow(window);
        SDL_Quit();

        return EXIT_FAILURE;
    }

    //Initialize OpenGL
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

    if (!initTextures())
    {
        std::cerr << "Unable to initialize textures" << std::endl;
        return false;
    }

    glEnable(GL_DEBUG_OUTPUT);
    glDebugMessageCallback(GlErrorCallback, 0);

    // Clear color buffer
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);

    return true;
}

/*
 * Initialize Shaders
 */
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


    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Compile vertex shader
    /*
    GLint status;
    char err_buf[512];

    vert_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vert_shader, 1, &vert_shader_src, NULL);
    glCompileShader(vert_shader);
    glGetShaderiv(vert_shader, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE) {
        glGetShaderInfoLog(vert_shader, sizeof(err_buf), NULL, err_buf);
        err_buf[sizeof(err_buf) - 1] = '\0';
        fprintf(stderr, "Vertex shader compilation failed: %s\n", err_buf);
        return 1;
    }
    */

    // Compile fragment shader
    /*
    frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(frag_shader, 1, &frag_shader_src, NULL);
    glCompileShader(frag_shader);
    glGetShaderiv(frag_shader, GL_COMPILE_STATUS, &status);
    if (status != GL_TRUE) {
        glGetShaderInfoLog(frag_shader, sizeof(err_buf), NULL, err_buf);
        err_buf[sizeof(err_buf) - 1] = '\0';
        fprintf(stderr, "Fragment shader compilation failed: %s\n", err_buf);
        return 1;
    }
    */

    // Link vertex and fragment shaders
    /*
    shader_prog = glCreateProgram();
    glAttachShader(shader_prog, vert_shader);
    glAttachShader(shader_prog, frag_shader);
    glBindFragDataLocation(shader_prog, 0, "out_Color");
    glLinkProgram(shader_prog);
    glUseProgram(shader_prog);
    */

    ar_param = shaderUtil->getUniformLocation("aspect_ratio");

    updateAspectRatio();

    return true;
}

/*
 * Initialize Geometry
 */
bool initGeometry()
{
    const GLfloat verts[6][4] = {
        //  x      y      s      t
        { -1.0f, -1.0f,  0.0f,  1.0f }, // BL
        { -1.0f,  1.0f,  0.0f,  0.0f }, // TL
        {  1.0f,  1.0f,  1.0f,  0.0f }, // TR
        {  1.0f, -1.0f,  1.0f,  1.0f }, // BR
    };

    const GLint indicies[] = {
        0, 1, 2, 0, 2, 3
    };

    // Populate vertex buffer
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);

    // Populate element buffer
    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indicies), indicies, GL_STATIC_DRAW);

    // Bind vertex position attribute
    GLint pos_attr_loc = glGetAttribLocation(shaderUtil->getProgramId(), "in_Position");
    glVertexAttribPointer(pos_attr_loc, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (void*)0);
    glEnableVertexAttribArray(pos_attr_loc);

    // Bind vertex texture coordinate attribute
    GLint tex_attr_loc = glGetAttribLocation(shaderUtil->getProgramId(), "in_Texcoord");
    glVertexAttribPointer(tex_attr_loc, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat), (void*)(2 * sizeof(GLfloat)));
    glEnableVertexAttribArray(tex_attr_loc);

    return true;
}

/*
 * Initialize textures
 */
bool initTextures()
{
    glGenTextures(1, &tex);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glUniform1i(shaderUtil->getUniformLocation("tex"), 0);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_INT_8_8_8_8_REV, logo_rgba);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    return true;
}

ImGuiIO& initImGui()
{
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
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

    /*
    glBindBuffer(GL_UNIFORM_BUFFER, gRaceDataBuffer);
    glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(GLfloat) * num_karts, race_data.data());
    glBufferSubData(GL_UNIFORM_BUFFER, sizeof(GLfloat) * 64, sizeof(int), &num_karts);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    */
}

void render()
{
    glClear(GL_COLOR_BUFFER_BIT);

    //Bind program
    shaderUtil->useProgram();

    /*
    //Enable vertex position
    glEnableVertexAttribArray(gVertexPos2DLocation);

    //Set vertex data
    glBindBuffer(GL_ARRAY_BUFFER, gVBO);
    glVertexAttribPointer(gVertexPos2DLocation, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), nullptr);

    //Set index data and render
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gIBO);
    glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_INT, nullptr);

    //Disable vertex position
    glDisableVertexAttribArray(gVertexPos2DLocation);
    */

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, NULL);

    //Unbind program
    shaderUtil->stopProgram();
}

void close()
{
	SDL_StopTextInput();

	// Cleanup ImGui
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	//glDeleteBuffers(1, &gRaceDataBuffer);

	shaderUtil->cleanUp();

    glDisableVertexAttribArray(0);
    glDeleteTextures(1, &tex);
    glDeleteBuffers(1, &ebo);
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    window = nullptr;
    SDL_Quit();
}

void updateAspectRatio()
{
    glViewport(0, 0, window_width, window_height);

    shaderUtil->useProgram();

    if (ar_param != -1)
    {
        float aspect_ratio = static_cast<float>(window_height) / window_width;
        glUniform1f(ar_param, aspect_ratio);
    }
    else {
        std::cerr << "Couldn't get fAspectRatio! " << std::endl;
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

    // 1. Show a simple window.
    // Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets automatically appears in a window called "Debug".
    {
        constexpr float value = 1234.f;
        char text1[128] = "";
        char text2[128] = "";

        ImGui::Text("Value = %f", value);                   // Display some text (you can use a format string too)
        ImGui::InputText("string1", text1, IM_ARRAYSIZE(text1));   // Input text with a label
        ImGui::Text("A second text object");                    // Another text object
        ImGui::InputText("string2", text2, IM_ARRAYSIZE(text2));// Another input text
    }

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

	for (int i=1; i<4; i++)
	{
		auto new_kart = universe->spawnGoKart(i);
		new_kart->setSpeed(DEFAULT_SPEED * i * 0.1f);

		glGenBuffers(1, &gRaceDataBuffer);
		glBindBuffer(GL_UNIFORM_BUFFER, gRaceDataBuffer);
		glBufferData(GL_UNIFORM_BUFFER, sizeof(float) * 64 + sizeof(int), nullptr, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_UNIFORM_BUFFER, 0);

		glBindBufferBase(GL_UNIFORM_BUFFER, 0, gRaceDataBuffer);
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
