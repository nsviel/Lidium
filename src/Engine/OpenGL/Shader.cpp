#include "Shader.h"

//Constructor / Destructor
Shader::Shader(){}
Shader::~Shader(){}

bool Shader::shaderManagment(string dir){
	if(configuration.VERBOSE_shader){
		cout<<"Shader management..."<<endl;
	}
	//---------------------------

	//Shaders path
	string vertexFilePath = dir + "shader.vs";
	string fragmentFilePath = dir + "shader.fs";
	//------------------------------------

	// Create the shaders program
	programID = glCreateProgram();
	if(configuration.VERBOSE_shader){
		cout<<"Shader program... ok"<<endl;
	}

	// Compile & check Shaders-
	list_Shader.clear();
	list_Shader.push_back(shader_compileVertex(vertexFilePath));
	list_Shader.push_back(shader_compileFragment(fragmentFilePath));
	if(configuration.VERBOSE_shader){
		cout<<"Shader compilation... ok"<<endl;
	}

	//Link program
	this->linkProgram();
	if(configuration.VERBOSE_shader){
		cout<<"Shader linkage... ok"<<endl;
	}

	//Detach shaders
	for(int i=0; i<list_Shader.size(); i++){
		this->detachShaders(list_Shader[i]);
	}
	if(configuration.VERBOSE_shader){
		cout<<"Shader cleaning... ok"<<endl;
	}

	//------------------------------------
	modelID = glGetUniformLocation(programID, "Model");
	mvpID = glGetUniformLocation(programID, "MVP");
	comID = glGetUniformLocation(programID, "COM");
	if(configuration.VERBOSE_shader){
		cout<<"Shader uniform... ok"<<endl;
	}

	//---------------------------
	return true;
}
GLuint Shader::shader_compileVertex(string file_path){
	GLuint shaderID = glCreateShader(GL_VERTEX_SHADER);
	//---------------------------

	//Read shader
	std::ifstream shaderStream(file_path.c_str(), std::ios::in);
	string shaderCode;
	if(shaderStream.is_open()){
		std::stringstream sstr;
		sstr << shaderStream.rdbuf();
		shaderCode = sstr.str();
		shaderStream.close();
	}

	//Compile shader
	int sucess, InfoLogLength;
	char const* sourcePointer = shaderCode.c_str();
	glShaderSource(shaderID, 1, &sourcePointer , NULL);
	glCompileShader(shaderID);
	glGetShaderiv(shaderID, GL_COMPILE_STATUS, &sucess);
	glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if(InfoLogLength > 0 || !sucess){
		cout<<"Shader Vertex loading error"<<endl;
	}

	//Bind to program
	glAttachShader(programID, shaderID);

	//---------------------------
	return shaderID;
}
GLuint Shader::shader_compileFragment(string file_path){
	GLuint shaderID = glCreateShader(GL_FRAGMENT_SHADER);
	//---------------------------

	//Read shader
	std::ifstream shaderStream(file_path.c_str(), std::ios::in);
	string shaderCode;
	if(shaderStream.is_open()){
		std::stringstream sstr;
		sstr << shaderStream.rdbuf();
		shaderCode = sstr.str();
		shaderStream.close();
	}

	//Compile shader
	int sucess, InfoLogLength;
	char const* sourcePointer = shaderCode.c_str();
	glShaderSource(shaderID, 1, &sourcePointer , NULL);
	glCompileShader(shaderID);
	glGetShaderiv(shaderID, GL_COMPILE_STATUS, &sucess);
	glGetShaderiv(shaderID, GL_INFO_LOG_LENGTH, &InfoLogLength);
	if(InfoLogLength > 0 || !sucess){
		cout<<"Shader Fragment loading error"<<endl;
	}

	//Bind to program
	glAttachShader(programID, shaderID);

	//---------------------------
	return shaderID;
}
bool Shader::detachShaders(GLuint shaderID){
	//glDetachShader(programID, shaderID);
	//glDeleteShader(shaderID);

	return true;
}
bool Shader::linkProgram(){
	//---------------------------

	// Link & check the program
	int sucess, InfoLogLength;
	glLinkProgram(programID);
	glGetProgramiv(programID, GL_LINK_STATUS, &sucess);
	glGetProgramiv(programID, GL_INFO_LOG_LENGTH, &InfoLogLength);

	if(InfoLogLength > 0 || !sucess){
		std::vector<char> ProgramErrorMessage(InfoLogLength+1);
		glGetProgramInfoLog(programID, InfoLogLength, NULL, &ProgramErrorMessage[0]);
		printf("%s\n", &ProgramErrorMessage[0]);
	}

	//---------------------------
	return true;
}
