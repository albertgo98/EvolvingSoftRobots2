//
//  VBO.h
//  PhysicsSimulator
//
//  Created by Albert Go on 10/31/21.
//

#ifndef VBO_CLASS_h
#define VBO_CLASS_h

#include <glad/glad.h>

class VBO
{
public:
    GLuint ID;
    VBO(GLfloat* vertices, GLsizeiptr size);
    
    void Bind();
    void Unbind();
    void Delete();
};


#endif /* VBO_h */
