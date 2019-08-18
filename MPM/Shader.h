#pragma once
#include <glad/glad.h> // include glad to get all the required OpenGL headers

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>


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
	// ------------------------------------------------------------------------
	void SetFloat(const std::string &name, float value) const
	{
		glUniform1f(glGetUniformLocation(m_ID, name.c_str()), value);
	}
	void SetVec(const std::string &name, glm::vec2 vec) const
	{
		glUniform2f(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y);
	}
	void SetVec(const std::string &name, glm::vec3 vec) const
	{
		glUniform3f(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y, vec.z);
	}
	void SetVec(const std::string &name, glm::vec4 vec) const
	{
		glUniform4f(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y, vec.z, vec.w);
	}
	void SetVec(const std::string &name, glm::uvec4 vec) const
	{
		glUniform4ui(glGetUniformLocation(m_ID, name.c_str()), vec.x, vec.y, vec.z, vec.w);
	}
	void SetMat(const std::string &name, glm::mat4 mat) const
	{
		glUniformMatrix4fv(glGetUniformLocation(m_ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(mat));
	}

	GLuint ID() const
	{
		return m_ID;
	}
	
protected:
	GLuint m_ID;

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
	StandardShader(const std::vector<std::string> &vertexPaths, const std::vector<std::string> &fragmentPaths, std::string headerPath)
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
			std::cout << "ERROR::SHADER::HAEDER_FILE_NOT_SUCCESFULLY_READ" << std::endl;
		}

		// 1. Vertex shader
		std::vector<GLuint> vertexShaders;
		vertexShaders.resize(vertexPaths.size());
		for (size_t i = 0; i < vertexPaths.size(); i++) {
			std::string vertexCode;
			std::ifstream vShaderFile;
			vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
			try
			{
				vShaderFile.open(vertexPaths[i]);

				for (std::string line; std::getline(vShaderFile, line); ) {
					if (line == "/*** HEADER ***/") {
						vertexCode += headerCode + "\n";
					}
					else {
						vertexCode += line + "\n";
					}
				}

				vShaderFile.close();
			}
			catch (std::ifstream::failure e)
			{
				if (!vShaderFile.eof()) {
					std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
				}
			}
			const char* vShaderCode = vertexCode.c_str();
			vertexShaders[i] = glCreateShader(GL_VERTEX_SHADER);
			glShaderSource(vertexShaders[i], 1, &vShaderCode, NULL);
			glCompileShader(vertexShaders[i]);
			checkCompileErrors(vertexShaders[i], "VERTEX");
		}


		// 2. Fragment shaders
		std::vector<GLuint> fragmentShaders;
		fragmentShaders.resize(fragmentPaths.size());
		for (size_t i = 0; i < fragmentPaths.size(); i++) {
			std::string fragPath = fragmentPaths[i];
			std::string fragCode;
			std::ifstream fShaderFile;
			fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
			try
			{
				fShaderFile.open(fragPath);
				std::stringstream fShaderStream;
				fShaderStream << fShaderFile.rdbuf();
				fShaderFile.close();
				fragCode = fShaderStream.str();
			}
			catch (std::ifstream::failure e)
			{
				std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
			}
			const char* fShaderCode = fragCode.c_str();
			fragmentShaders[i] = glCreateShader(GL_FRAGMENT_SHADER);
			glShaderSource(fragmentShaders[i], 1, &fShaderCode, NULL);
			glCompileShader(fragmentShaders[i]);
			checkCompileErrors(fragmentShaders[i], "FRAGMENT");
		}

		// 3. Create program, link
		// shader Program
		m_ID = glCreateProgram();
		for (GLuint vert : vertexShaders) {
			glAttachShader(m_ID, vert);
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

class Shader
{
public:
	unsigned int ID;
	// constructor generates the shader on the fly
	// ------------------------------------------------------------------------
	Shader(const std::string &vertexPath, const std::string &fragmentPath)
	{
		// 1. retrieve the vertex/fragment source code from filePath
		std::string vertexCode;
		std::string fragmentCode;
		std::ifstream vShaderFile;
		std::ifstream fShaderFile;
		// ensure ifstream objects can throw exceptions:
		vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		try
		{
			// open files
			vShaderFile.open(vertexPath);
			fShaderFile.open(fragmentPath);
			std::stringstream vShaderStream, fShaderStream;
			// read file's buffer contents into streams
			vShaderStream << vShaderFile.rdbuf();
			fShaderStream << fShaderFile.rdbuf();
			// close file handlers
			vShaderFile.close();
			fShaderFile.close();
			// convert stream into string
			vertexCode = vShaderStream.str();
			fragmentCode = fShaderStream.str();
		}
		catch (std::ifstream::failure e)
		{
			std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ" << std::endl;
		}
		const char* vShaderCode = vertexCode.c_str();
		const char * fShaderCode = fragmentCode.c_str();
		// 2. compile shaders
		unsigned int vertex, fragment;
		// vertex shader
		vertex = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertex, 1, &vShaderCode, NULL);
		glCompileShader(vertex);
		checkCompileErrors(vertex, "VERTEX");
		// fragment Shader
		fragment = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fragment, 1, &fShaderCode, NULL);
		glCompileShader(fragment);
		checkCompileErrors(fragment, "FRAGMENT");
		// shader Program
		ID = glCreateProgram();
		glAttachShader(ID, vertex);
		glAttachShader(ID, fragment);
		glLinkProgram(ID);
		checkCompileErrors(ID, "PROGRAM");
		// delete the shaders as they're linked into our program now and no longer necessary
		glDeleteShader(vertex);
		glDeleteShader(fragment);
	}
	// activate the shader
	// ------------------------------------------------------------------------
	void use()
	{
		glUseProgram(ID);
	}
	// utility uniform functions
	// ------------------------------------------------------------------------
	void setBool(const std::string &name, bool value) const
	{
		glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
	}
	// ------------------------------------------------------------------------
	void setInt(const std::string &name, int value) const
	{
		glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
	}
	// ------------------------------------------------------------------------
	void setFloat(const std::string &name, float value) const
	{
		glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
	}
	void setVec(const std::string &name, glm::vec2 vec) const
	{
		glUniform2f(glGetUniformLocation(ID, name.c_str()), vec.x, vec.y);
	}
	void setVec(const std::string &name, glm::vec3 vec) const
	{
		glUniform3f(glGetUniformLocation(ID, name.c_str()), vec.x, vec.y, vec.z);
	}
	void setVec(const std::string &name, glm::vec4 vec) const
	{
		glUniform4f(glGetUniformLocation(ID, name.c_str()), vec.x, vec.y, vec.z, vec.w);
	}
	void setMat(const std::string &name, glm::mat4 mat) const
	{
		glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, glm::value_ptr(mat));
	}
private:
	// utility function for checking shader compilation/linking errors.
	// ------------------------------------------------------------------------
	void checkCompileErrors(unsigned int shader, std::string type)
	{
		int success;
		char infoLog[1024];
		if (type != "PROGRAM")
		{
			glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
			if (!success)
			{
				glGetShaderInfoLog(shader, 1024, NULL, infoLog);
				std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
			}
		}
		else
		{
			glGetProgramiv(shader, GL_LINK_STATUS, &success);
			if (!success)
			{
				glGetProgramInfoLog(shader, 1024, NULL, infoLog);
				std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
			}
		}
	}
};