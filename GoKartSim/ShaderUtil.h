#pragma once
#include <string>
#include <glew.h>

class ShaderUtil
{
public:
	ShaderUtil() = default;
	~ShaderUtil() = default;

	bool load(const std::string& vertexShaderFile, const std::string& fragmentShaderFile);
	void useProgram();
	void stopProgram();
	void deleteProgram();
	void getAttribLocation(const GLchar* attribute_name, GLint& location_out);
	GLuint getProgramId() const;
private:
	unsigned int GetCompiledShader(unsigned int shader_type, const std::string& shader_source);
	GLuint mProgramId;
};