#pragma once

#include "Constants.h"

#include <glad/glad.h> // include glad to get all the required OpenGL headers

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>


class AdvancedShader {
public:

	~AdvancedShader() {
		glDeleteProgram(m_ID);
	}

	
	// activate the shader
	// ------------------------------------------------------------------------
	void Use()
	{
		glUseProgram(m_ID);
	}
	// utility uniform functions
	// ------------------------------------------------------------------------
	void SetBool(const std::string &name, bool value) const
	{
		glUniform1i(glGetUniformLocation(m_ID, name.c_str()), (int)value);
	}
	// ------------------------------------------------------------------------
	void SetInt(const std::string &name, int value) const
	{
		glUniform1i(glGetUniformLocation(m_ID, name.c_str()), value);
	}
	void SetuInt(const std::string& name, unsigned int value) const
	{
		glUniform1ui(glGetUniformLocation(m_ID, name.c_str()), value);
	}
	void SetVec(const std::string& name, glm::uvec4 vec) const
	{
		glUniform4ui(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y, vec.z, vec.w);
	}
	// ------------------------------------------------------------------------

	// floats
	void SetReal(const std::string& name, float value) const
	{
		glUniform1f(glGetUniformLocation(m_ID, name.c_str()), value);
	}
	void SetVec(const std::string &name, glm::highp_fvec2 vec) const
	{
		glUniform2f(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y);
	}
	void SetVec(const std::string &name, glm::highp_fvec3 vec) const
	{
		glUniform3f(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y, vec.z);
	}
	void SetVec(const std::string &name, glm::highp_fvec4 vec) const
	{
		glUniform4f(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y, vec.z, vec.w);
	}
	void SetMat(const std::string &name, glm::highp_fmat4 mat) const
	{
		glUniformMatrix4fv(glGetUniformLocation(m_ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(mat));
	}

	// doubles
	void SetReal(const std::string& name, double value) const
	{
		glUniform1d(glGetUniformLocation(m_ID, name.c_str()), value);
	}
	void SetVec(const std::string& name, glm::highp_dvec2 vec) const
	{
		glUniform2d(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y);
	}
	void SetVec(const std::string& name, glm::highp_dvec3 vec) const
	{
		glUniform3d(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y, vec.z);
	}
	void SetVec(const std::string& name, glm::highp_dvec4 vec) const
	{
		glUniform4d(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y, vec.z, vec.w);
	}
	void SetMat(const std::string& name, glm::highp_dmat2 mat) const
	{
		glUniformMatrix2dv(glGetUniformLocation(m_ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(mat));
	}
	void SetMat(const std::string& name, glm::highp_dmat4 mat) const
	{
		glUniformMatrix4dv(glGetUniformLocation(m_ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(mat));
	}

	GLuint ID() const
	{
		return m_ID;
	}
	
protected:
	GLuint m_ID;

	bool CompileShaders(std::vector<GLuint>& shaders, const std::vector<std::string>& shaderPaths, const std::string &headerCode, GLenum shaderType) {
		shaders.resize(shaderPaths.size());
		for (size_t i = 0; i < shaderPaths.size(); i++) {
			std::string shaderCode;
			std::ifstream shaderFile;
			shaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
			try
			{
				shaderFile.open(shaderPaths[i]);

				for (std::string line; std::getline(shaderFile, line); ) {
					if (line == "/*** HEADER ***/") {
						shaderCode += headerCode + "\n";
					}
					else {
						shaderCode += line + "\n";
					}
				}

				shaderFile.close();
			}
			catch (std::ifstream::failure e)
			{
				if (!shaderFile.eof()) {
					std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
					return false;
				}
			}
			const char* cShaderCode = shaderCode.c_str();
			shaders[i] = glCreateShader(shaderType);
			glShaderSource(shaders[i], 1, &cShaderCode, NULL);
			glCompileShader(shaders[i]);
			checkCompileErrors(shaders[i], shaderPaths[i]);
		}
		return true;
	}

	// utility function for checking shader compilation/linking errors.
	// ------------------------------------------------------------------------
	void checkCompileErrors(GLuint ID, std::string type)
	{
		int success;
		char infoLog[1024];
		if (type != "PROGRAM")
		{
			glGetShaderiv(ID, GL_COMPILE_STATUS, &success);
			if (!success)
			{
				glGetShaderInfoLog(ID, 1024, NULL, infoLog);
				std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
			}
		}
		else
		{
			glGetProgramiv(ID, GL_LINK_STATUS, &success);
			if (!success)
			{
				glGetProgramInfoLog(ID, 1024, NULL, infoLog);
				std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
			}
		}
	}
};

class StandardShader : public AdvancedShader {
public:
	StandardShader(const std::vector<std::string> &vertexPaths, const std::vector<std::string> &geometryPaths, const std::vector<std::string> &fragmentPaths, std::string headerPath)
	{
		// First get the header file str into a std::string
		std::string headerCode;
		std::ifstream headerFile;
		headerFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		try
		{
			headerFile.open(headerPath);
			std::stringstream headerStream;
			headerStream << headerFile.rdbuf();
			headerFile.close();
			headerCode = headerStream.str();
		}
		catch (std::ifstream::failure e) {
			std::cout << "ERROR::SHADER::HEADER_FILE_NOT_SUCCESFULLY_READ" << std::endl;
		}

		// 1. Vertex shader
		std::vector<GLuint> vertexShaders;
		CompileShaders(vertexShaders, vertexPaths, headerCode, GL_VERTEX_SHADER);

		// 1.5 Geometry shaders
		std::vector<GLuint> geometryShaders;
		CompileShaders(geometryShaders, geometryPaths, headerCode, GL_GEOMETRY_SHADER);

		// 2. Fragment shaders
		std::vector<GLuint> fragmentShaders;
		CompileShaders(fragmentShaders, fragmentPaths, headerCode, GL_FRAGMENT_SHADER);

		// 3. Create program, link
		// shader Program
		m_ID = glCreateProgram();
		for (GLuint vert : vertexShaders) {
			glAttachShader(m_ID, vert);
		}
		for (GLuint geo : geometryShaders) {
			glAttachShader(m_ID, geo);
		}
		for (GLuint frag : fragmentShaders) {
			glAttachShader(m_ID, frag);
		}
		glLinkProgram(m_ID);
		checkCompileErrors(m_ID, "PROGRAM");

		// delete the shaders as they're linked into our program now and no longer necessary
		for (GLuint vert : vertexShaders) {
			glDeleteShader(vert);
		}
		for (GLuint geo : geometryShaders) {
			glDeleteShader(geo);
		}
		for (GLuint frag : fragmentShaders) {
			glDeleteShader(frag);
		}
	}
};

class ComputeShader : public AdvancedShader {
public:
	ComputeShader(std::vector<std::string> computePaths, std::string headerPath) {
		// Constructs compute shader using compute paths? and a header file

		// First get the header file str into a std::string
		std::string headerCode;
		std::ifstream headerFile;
		headerFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		try
		{
			headerFile.open(headerPath);
			std::stringstream headerStream;
			headerStream << headerFile.rdbuf();
			headerFile.close();
			headerCode = headerStream.str();
		}
		catch (std::ifstream::failure e) {
			std::cout << "ERROR::SHADER::HAEDER_FILE_NOT_SUCCESFULLY_READ" << std::endl;
		}

		std::vector<GLuint> computes;
		computes.resize(computePaths.size());
		for (size_t i = 0; i < computePaths.size(); i++) {
			std::string compPath = computePaths[i];
			std::string compCode;
			std::ifstream cShaderFile;
			cShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
			try
			{
				cShaderFile.open(compPath);
				/*std::stringstream cShaderStream;
				cShaderStream << cShaderFile.rdbuf();*/
				
				for (std::string line; std::getline(cShaderFile, line); ) {
					if (line == "/*** HEADER ***/") {
						compCode += headerCode + "\n";
					}
					else {
						compCode += line + "\n";
					}
				}
				
				cShaderFile.close();
				//compCode = cShaderStream.str();
			}
			catch (std::ifstream::failure e)
			{
				// only an error if it did not reach end of file
				if (!cShaderFile.eof()) {
					std::cout << compCode << std::endl;
					std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
					std::cout << e.code() << std::endl;
					std::cout << e.what() << std::endl;
				}
			}
			const char* cShaderCode = compCode.c_str();
			computes[i] = glCreateShader(GL_COMPUTE_SHADER);
			glShaderSource(computes[i], 1, &cShaderCode, NULL);
			glCompileShader(computes[i]);
			checkCompileErrors(computes[i], "COMPUTE");
		}

		// shader Program
		m_ID = glCreateProgram();
		for (GLuint comp : computes) {
			glAttachShader(m_ID, comp);
		}
		glLinkProgram(m_ID);
		checkCompileErrors(m_ID, "PROGRAM");
		for (GLuint comp : computes) {
			glDeleteShader(comp);
		}
	}
};
