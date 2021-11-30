//
//  EBO.h
//  PhysicsSimulator
//
//  Created by Albert Go on 10/31/21.
//

#ifndef EBO_CLASS_h
#define EBO_CLASS_h

#include <glad/glad.h>

class EBO
{
public:
    GLuint ID;
    EBO(GLuint* indices, GLsizeiptr size);
    
    void Bind();
    void Unbind();
    void Delete();
};

#endif /* EBO_h */
