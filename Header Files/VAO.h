//
//  VAO.h
//  PhysicsSimulator
//
//  Created by Albert Go on 10/31/21.
//

#ifndef VAO_CLASS_h
#define VAO_CLASS_h

#include <glad/glad.h>
#include "VBO.h"

class VAO
{
public:
    GLuint ID;
    VAO();
    
//    void LinkVBO(VBO VBO, GLuint layout);
    void LinkAttrib(VBO VBO, GLuint layout, GLuint numComponents, GLenum type, GLsizeiptr stride, void* offset);
    
    void Bind();
    void Unbind();
    void Delete();
};

#endif /* VAO_h */
