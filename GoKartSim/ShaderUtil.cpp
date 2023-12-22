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
	glGetProgramiv(mProgramId_, GL_INFO_LOG_LENGTH, &length);

	std::string logs;
	logs.resize(length);
	glGetProgramInfoLog(mProgramId_, length, &length, &logs[0]);
	throw std::exception{ std::format("{} in program {}: \"{}\"\n", message, mProgramId_, logs).c_str() };
}

bool ShaderUtil::load(const std::string& vertexShaderFile, const std::string& fragmentShaderFile, const std::string& geometryShaderFile)
{
	mProgramId_ = glCreateProgram();

	unsigned int vs = GetCompiledShader(GL_VERTEX_SHADER, vertexShaderFile);
	unsigned int fs = GetCompiledShader(GL_FRAGMENT_SHADER, fragmentShaderFile);
	unsigned int gs = GetCompiledShader(GL_GEOMETRY_SHADER, geometryShaderFile);

	if (vs)
	{
		glAttachShader(mProgramId_, vs);
	}

	if (fs)
	{
		glAttachShader(mProgramId_, fs);
	}

	if (gs)
	{
		glAttachShader(mProgramId_, gs);
	}

	glLinkProgram(mProgramId_);

	GLint linked;
	glGetProgramiv(mProgramId_, GL_LINK_STATUS, &linked);

	if (!linked)
	{
		handleProgramError("Linker error");
	}


	glValidateProgram(mProgramId_);

	glDeleteShader(vs);
	glDeleteShader(fs);
	glDeleteShader(gs);

	return true;
}

void ShaderUtil::useProgram()
{
	glUseProgram(mProgramId_);
}

void ShaderUtil::stopProgram()
{
	glUseProgram(NULL);
}

GLint ShaderUtil::getAttribLocation(const GLchar* attribute_name)
{
	GLint attribLocation = glGetAttribLocation(mProgramId_, attribute_name);
	return attribLocation;
}

void ShaderUtil::deleteProgram()
{
	glDeleteProgram(mProgramId_);
}

GLuint ShaderUtil::getProgramId() const
{
	return mProgramId_;
}