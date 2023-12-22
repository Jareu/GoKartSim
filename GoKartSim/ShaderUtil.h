#pragma once
#include <string>
#include <glew.h>

class ShaderUtil
{
public:
	ShaderUtil() = default;
	~ShaderUtil() = default;

	bool load(const std::string& vertexShaderFile, const std::string& fragmentShaderFile, const std::string& geometryShaderFile);
	void useProgram();
	void stopProgram();
	void cleanUp();
	GLint getAttribLocation(const GLchar* attribute_name);
	GLuint getProgramId() const;
private:
	unsigned int GetCompiledShader(unsigned int shader_type, const std::string& shader_source);
	void handleProgramError(const std::string& message);
	GLuint shader_program_;
	GLuint vert_shader_;
	GLuint frag_shader_;
	GLuint geom_shader_;
};