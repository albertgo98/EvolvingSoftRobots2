//
//  VAO.cpp
//  PhysicsSimulator
//
//  Created by Albert Go on 10/31/21.
//

#include <stdio.h>
#include "VAO.h"

VAO::VAO()
{
    glGenVertexArrays(1, &ID);
    
}

//void VAO::LinkVBO(VBO VBO, GLuint layout)
void VAO::LinkAttrib(VBO VBO, GLuint layout, GLuint numComponents, GLenum type, GLsizeiptr stride, void* offset)
{
    VBO.Bind();
//    glVertexAttribPointer(layout, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), (void*)0);
    glVertexAttribPointer(layout, numComponents, type, GL_FALSE, stride, offset);
    glEnableVertexAttribArray(layout);
    VBO.Unbind();
}

void VAO::Bind()
{
    glBindVertexArray(ID);
}

void VAO::Unbind()
{
    glBindVertexArray(0);
}

void VAO::Delete()
{
    glDeleteVertexArrays(1, &ID);
}
