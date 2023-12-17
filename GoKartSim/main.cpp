#include <iostream>

#include "main.h"

#include "globals.h"

bool init()
{
	//Initialize SDL
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
	{
		std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
		return false;
	}

	//Use OpenGL 3.1 core
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

	//Create window
	window = SDL_CreateWindow("GoKart Simulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, window_width, window_height, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);

	if (window == nullptr)
	{
		std::cerr << "Window could not be created! SDL Error:" << SDL_GetError() << std::endl;
		return false;
	}

	//Create context
	gl_context = SDL_GL_CreateContext(window);
	SDL_GL_MakeCurrent(window, gl_context);
	SDL_GL_SetSwapInterval(1); // Enable vsync


	if (gl_context == nullptr)
	{
		std::cerr << "OpenGL context could not be created! SDL Error:" << SDL_GetError() << std::endl;
		return false;
	}

	//Initialize GLEW
	glewExperimental = GL_TRUE;
	GLenum glewError = glewInit();

	if (glewError != GLEW_OK)
	{
		std::cerr << "Error initializing GLEW!" << glewGetErrorString(glewError) << std::endl;
	}

	//Use Vsync
	if (SDL_GL_SetSwapInterval(1) < 0)
	{
		std::cerr << "Warning: Unable to set VSync! SDL Error:" << SDL_GetError() << std::endl;
	}

	//Initialize OpenGL
	if (!initGL())
	{
		std::cerr << "Unable to initialize OpenGL!" << std::endl;
		return false;
	}

	return true;
}

bool initGL()
{
	// load shaders
	try {
		shaderUtil.load("vs.shader", "fs.shader", "gs.shader");
	}
	catch (std::exception e)
	{
		std::cerr << e.what() << std::endl;
		return false;
	}

	// Clear color buffer
	glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);

	//VBO data
	GLfloat vertexData[] = {
		0.0f, 0.0f, 1.0f, 1.0f, 0.0f, 32.f
	};

	//IBO data
	GLuint indexData[] = { 0 };

	//Create VBO
	glGenBuffers(1, &gVBO);
	glBindBuffer(GL_ARRAY_BUFFER, gVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexData), vertexData, GL_STATIC_DRAW);

	// Create VAO
	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	// Specify layout of point data
	// TODO: don't do this here. make race_data_size = 1 to begin with, and recalculate glVertexAttribPointer data when new karts are added to race
	int number_of_karts = 0;
	auto numKartsAttrib = shaderUtil.getAttribLocation("number_of_karts");
	int position = 0;
	glEnableVertexAttribArray(numKartsAttrib);
	glVertexAttribPointer(numKartsAttrib, 1, GL_FLOAT, GL_FALSE, sizeof(float), (void*)(position * sizeof(float)));

	size_t race_data_size = (number_of_karts + 1) * sizeof(float);
	position = 1;
	auto raceDataAttrib = shaderUtil.getAttribLocation("race_Data");
	glEnableVertexAttribArray(raceDataAttrib);
	glVertexAttribPointer(raceDataAttrib, number_of_karts, GL_FLOAT, GL_FALSE, race_data_size, (void*)(position * sizeof(float)));
	glBindVertexArray(0);

	//Create IBO
	glGenBuffers(1, &gIBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gIBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 4 * sizeof(GLuint), indexData, GL_STATIC_DRAW);

	ar_param = glGetUniformLocation(shaderUtil.getProgramId(), "fAspectRatio");

	updateAspectRatio();
	
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
		gRenderQuad = !gRenderQuad;
	}
}

void update()
{
	//No per frame update needed
	universe->tick();
	auto race_data = universe->getRaceData();
	std::vector<GLfloat> gl_race_data {};
	gl_race_data.emplace_back(static_cast<float> (race_data.size())); // number of karts
	gl_race_data.insert(gl_race_data.end(), race_data.begin(), race_data.end()); // array of kart positions

	glBindBuffer(GL_ARRAY_BUFFER, gVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * gl_race_data.size(), gl_race_data.data(), GL_STATIC_DRAW);
}

void render()
{
	glClear(GL_COLOR_BUFFER_BIT);

	//Render quad
	if (gRenderQuad)
	{
		//Bind program
		shaderUtil.useProgram();
		
		glDrawArrays(GL_POINTS, 0, 4);

		//Unbind program
		shaderUtil.stopProgram();
	}
}

void close()
{
	//Disable text input
	SDL_StopTextInput();

	// Cleanup ImGui
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplSDL2_Shutdown();
	ImGui::DestroyContext();

	shaderUtil.deleteProgram();

	SDL_GL_DeleteContext(gl_context);

	//Destroy window	
	SDL_DestroyWindow(window);
	window = nullptr;

	//Quit SDL subsystems
	SDL_Quit();

}

void updateAspectRatio()
{
	shaderUtil.useProgram();

	if (ar_param != -1)
	{
		float aspect_ratio = static_cast<float>(window_height) / window_width;
		glUniform1f(ar_param, aspect_ratio);
	}
	else {
		std::cerr << "Couldn't get fAspectRatio! " << std::endl;
	}

	shaderUtil.stopProgram();
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
	SDL_GL_SwapWindow(window);
}

int main(int argc, char* argv[])
{
	if (init() == false)
	{
		std::cerr << "Initialization failed!" << std::endl;
		return EXIT_FAILURE;
	}

	auto& io = initImGui();
	universe = std::make_unique<Universe>(SEED);
    universe->spawnGoKart(12);
	  
    // Main loop
	SDL_StartTextInput();

    while (!quit) {
		handleEvents();

		update();
         
        // Draw something
		render();
		
		renderUi(io);
    }

	close();

    return EXIT_SUCCESS;
}
