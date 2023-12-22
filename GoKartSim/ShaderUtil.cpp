#include "ShaderUtil.h"

#include <iostream>
#include <fstream>
#include <format>

unsigned int ShaderUtil::GetCompiledShader(unsigned int shader_type, const std::string& shader_filename)
{
	if (shader_filename.empty()) {
		return GL_FALSE;
	}

	std::ifstream shader_file(shader_filename);
	const std::string shader_source((std::istreambuf_iterator<char>(shader_file)), std::istreambuf_iterator<char>());

	unsigned int shader_id = glCreateShader(shader_type);

	const char* c_source = shader_source.c_str();
	glShaderSource(shader_id, 1, &c_source, nullptr);
	glCompileShader(shader_id);

	GLint result;
	glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);

	if (result == GL_FALSE)
	{
		int length;
		glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &length);

		std::string logs;
		logs.resize(length);
		glGetShaderInfoLog(shader_id, length, NULL, &logs[0]);

		throw std::exception{ std::format("Compilation error in shader: {}\n", logs).c_str()};
	}

	return shader_id;
}

void ShaderUtil::handleProgramError(const std::string& message)
{
	int length;
	glGetProgramiv(shader_program_, GL_INFO_LOG_LENGTH, &length);

	std::string logs;
	logs.resize(length);
	glGetProgramInfoLog(shader_program_, length, &length, &logs[0]);
	throw std::exception{ std::format("{} in program {}: \"{}\"\n", message, shader_program_, logs).c_str() };
}

bool ShaderUtil::load(const std::string& vertexShaderFile, const std::string& fragmentShaderFile, const std::string& geometryShaderFile)
{
	shader_program_ = glCreateProgram();

	vert_shader_ = GetCompiledShader(GL_VERTEX_SHADER, vertexShaderFile);
	frag_shader_ = GetCompiledShader(GL_FRAGMENT_SHADER, fragmentShaderFile);
	geom_shader_ = GetCompiledShader(GL_GEOMETRY_SHADER, geometryShaderFile);

	if (vert_shader_)
	{
		glAttachShader(shader_program_, vert_shader_);
	}

	if (frag_shader_)
	{
		glAttachShader(shader_program_, frag_shader_);
	}

	if (geom_shader_)
	{
		glAttachShader(shader_program_, geom_shader_);
	}

	glLinkProgram(shader_program_);

	GLint linked;
	glGetProgramiv(shader_program_, GL_LINK_STATUS, &linked);

	if (!linked)
	{
		handleProgramError("Linker error");
	}


	glValidateProgram(shader_program_);

	glDeleteShader(vert_shader_);
	glDeleteShader(frag_shader_);
	glDeleteShader(geom_shader_);

	return true;
}

void ShaderUtil::useProgram()
{
	glUseProgram(shader_program_);
}

void ShaderUtil::stopProgram()
{
	glUseProgram(NULL);
}

GLint ShaderUtil::getAttribLocation(const GLchar* attribute_name)
{
	GLint attribLocation = glGetAttribLocation(shader_program_, attribute_name);
	return attribLocation;
}

GLint ShaderUtil::getUniformLocation(const GLchar* uniform_name)
{
	GLint uniformLocation = glGetUniformLocation(shader_program_, uniform_name);
	return uniformLocation;
}

void ShaderUtil::cleanUp()
{
	glUseProgram(0);
	glDisableVertexAttribArray(0);
	glDetachShader(shader_program_, vert_shader_);
	glDetachShader(shader_program_, frag_shader_);
	glDetachShader(shader_program_, geom_shader_);
	glDeleteProgram(shader_program_);
}

GLuint ShaderUtil::getProgramId() const
{
	return shader_program_;
}