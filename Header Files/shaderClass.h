//
//  shaderClass.h
//  PhysicsSimulator
//
//  Created by Albert Go on 10/31/21.
//

#ifndef SHADER_CLASS_h
#define SHADER_CLASS_h

#include <glad/glad.h>
#include<string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cerrno>

std::string get_file_contents(const char* filename);

class Shader
{
    public:
        GLuint ID;
        Shader(const char* vertexFile, const char* fragmentFile);
    
        void Activate();
        void Delete();
};

#endif /* shaderClass_h */
