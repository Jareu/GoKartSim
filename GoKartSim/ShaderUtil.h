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
	void deleteProgram();
	GLint getAttribLocation(const GLchar* attribute_name);
	GLuint getProgramId() const;
private:
	unsigned int GetCompiledShader(unsigned int shader_type, const std::string& shader_source);
	GLuint mProgramId;
};