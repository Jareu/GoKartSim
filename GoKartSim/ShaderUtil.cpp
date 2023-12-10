#include "ShaderUtil.h"

#include <iostream>
#include <fstream>
#include <format>

unsigned int ShaderUtil::GetCompiledShader(unsigned int shader_type, const std::string& shader_source)
{
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

bool ShaderUtil::load(const std::string& vertexShaderFile, const std::string& fragmentShaderFile)
{
	std::ifstream is_vs(vertexShaderFile);
	const std::string f_vs((std::istreambuf_iterator<char>(is_vs)), std::istreambuf_iterator<char>());

	std::ifstream is_fs(fragmentShaderFile);
	const std::string f_fs((std::istreambuf_iterator<char>(is_fs)), std::istreambuf_iterator<char>());

	mProgramId = glCreateProgram();

	unsigned int vs = GetCompiledShader(GL_VERTEX_SHADER, f_vs);
	unsigned int fs = GetCompiledShader(GL_FRAGMENT_SHADER, f_fs);

	glAttachShader(mProgramId, vs);
	glAttachShader(mProgramId, fs);

	glLinkProgram(mProgramId);
	glValidateProgram(mProgramId);

	GLint programSuccess = GL_TRUE;
	glGetProgramiv(mProgramId, GL_LINK_STATUS, &programSuccess);
	if (programSuccess != GL_TRUE)
	{
		int length;
		glGetProgramiv(mProgramId, GL_INFO_LOG_LENGTH, &length);

		auto strInfoLog = std::make_unique<GLchar[]>(length + 1);
		glGetProgramInfoLog(mProgramId, length, &length, strInfoLog.get());

		throw std::exception{ std::format("Error linking program %d: %s\n", mProgramId, strInfoLog.get()).c_str() };
	}

	glDeleteShader(vs);
	glDeleteShader(fs);

	return true;
}

void ShaderUtil::useProgram()
{
	glUseProgram(mProgramId);
}

void ShaderUtil::stopProgram()
{
	glUseProgram(NULL);
}

void ShaderUtil::getAttribLocation(const GLchar* attribute_name, GLint& location_out)
{
	location_out = glGetAttribLocation(mProgramId, attribute_name);
}

void ShaderUtil::deleteProgram()
{
	glDeleteProgram(mProgramId);
}

GLuint ShaderUtil::getProgramId() const
{
	return mProgramId;
}