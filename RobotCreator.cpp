//
//  main.cpp
//  PhysicsSimulator
//
//  Created by Albert Go on 10/31/21.
//

#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <math.h>
#include <numeric>
#include <list>
#include <algorithm>

#include "shaderClass.h"
#include "VAO.h"
#include "VAO.h"
#include "EBO.h"
//#include "Camera.h"
using namespace std;

struct PointMass{
    double mass;
    vector<float> position; // {x, y, z}
    vector<float> velocity; // {v_x, v_y, v_z}
    vector<float> acceleration; // {a_x, a_y, a_z}
    vector<float> forces; // {f_x, f_y, f_z}
    int ID; //index of where a particular mass lies in the robot.masses vector https://forms.gle/bKtGGQKmbtsS6kSV7
    
};

struct Spring{
    float L0; // resting length
    float L; // current length
    float k; // spring constant
    int m0; // connected to which PointMass
    int m1; // connected to which PointMass
    float original_L0;
    int ID; //the index of where a particular string lies in the robot.springs vector
};

struct Cube{
    vector<PointMass> masses;
    vector<Spring> springs;
    vector<int> joinedCubes;
    vector<int> otherFaces; //faces of other cubes that are joined to it
    vector<int> joinedFaces; //faces of the cube that are joined to other cubes
    vector<int> massIDs; //where the verteces of the cube correspond to the Robot.masses vector
    vector<int> springIDs; //where the springs of the cube correspond to the Robot.springs vector
    vector<int> free_faces;
    vector<float> center;
};

struct Robot{
    vector<PointMass> masses; //vector of masses that make up the robot
    vector<Spring> springs; //vector of springs that make up the robot
    vector<int> cubes;
    vector<Cube> all_cubes;
    vector<int> available_cubes;
};

struct Equation{
    float k;
    float a;
    float w;
    float c;
};

struct Controller{
    vector<Equation> motor;
    vector<float> start;
    vector<float> end;
    float fitness;
};

const double g = -9.81; //acceleration due to gravity
const double b = 1; //damping (optional) Note: no damping means your cube will bounce forever
const float spring_constant = 5000.0f; //this worked best for me given my dt and mass of each PointMass
const float mu_s = 0.74; //coefficient of static friction
const float mu_k = 0.57; //coefficient of kinetic friction
float T = 0.0;
float dt = 0.0001;
bool breathing = false;

vector<int> face0 = {0, 1, 2, 3}; //face 0 (bottom face) corresponds with these cube vertices; only connects with face 5
vector<int> face1 = {0, 3, 4, 7}; //face 1(front face) corresponds with these cube vertices; only connects with face 3
vector<int> face2 = {0, 1, 4, 5}; //face 2 (left face) corresponds with these cube vertices; only connects with face 4
vector<int> face3 = {1, 2, 5, 6}; //face 3 (back face) corrresponds with these cube vertices; only connects with face 1
vector<int> face4 = {3, 2, 7, 6}; //face 4 (right face) corresponds with these cube vertices; only conncects with face 2
vector<int> face5 = {4, 5, 6, 7}; //face 5 (top face) corresponds with these cube vertices; only connects with face 0

vector<int> face0_springs = {0, 1, 2, 3, 4, 5}; //face 0 (bottom face) corresponds with these cube springs; only connects with face 5
vector<int> face1_springs = {3, 6, 9, 10, 11, 21}; //face 1 (front face) corresponds with these cube springs; only connects with face 3
vector<int> face2_springs = {0, 6, 7, 12, 13, 18}; //face 2 (left face) corresponds with these cube springs; only connects with face 4
vector<int> face3_springs = {1, 7, 8, 14, 15, 19}; //face 3 (back face) corresponds with these cube springs; only connects with face 1
vector<int> face4_springs = {2, 9, 8, 17, 16, 20}; //face 4 (right face) corresponds with these cube springs; only connects with face 2
vector<int> face5_springs = {18, 19, 20, 21, 22, 23}; // face 5 (top face) corresponds with these cube springs; only connects with face 0

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void initialize_masses(vector<PointMass> &masses);
void initialize_springs(vector<Spring> &springs);
void apply_force(vector<PointMass> &masses);
void update_pos_vel_acc(Robot &robot);
void update_forces(Robot &robot);
void reset_forces(Robot &robot);
void update_breathing(Robot &robot, Controller &control);
void initialize_robot(Robot &robot);
void initialize_cube(Cube &cube);
void initialize_controller(Controller &control);
void fuse_faces(Cube &cube1, Cube &cube2, int cube1_index, int cube2_index, vector<PointMass> &masses, vector<Spring> &springs, int combine1, int combine2, vector<int> &masses_left, vector<int> &springs_left);


const unsigned int width = 1000;
const unsigned int height = 1000;

// camera
glm::vec3 cameraPos   = glm::vec3(10.0f, 2.0f, 3.0f); //move the camera such that it is facing the cube
glm::vec3 cameraFront = glm::vec3(0.0f, 1.0f, 0.0f); //make the front of the camera looking towards the positive y axis direction
glm::vec3 cameraUp    = glm::vec3(0.0f, 0.0f, 1.0f); //make the camera such that the z axis is going up

bool firstMouse = true;
bool firstClick = true;
float yaw   = -45.0f;    // yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch =  0.0f;
float lastX =  800.0f / 2.0;
float lastY =  600.0 / 2.0;
float fov   =  60.0f;

// timing
float deltaTime = 0.0f;    // time between current frame and last frame
float lastFrame = 0.0f;

void processInput(GLFWwindow *window)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
    
    float cameraSpeed = 2.5 * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        cameraPos += cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        cameraPos -= cameraSpeed * cameraFront;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if(glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS){
        cameraPos += cameraSpeed * cameraUp;
    }
    if(glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS){
        cameraPos -= cameraSpeed * cameraUp;
    }
}

int main(int argc, const char * argv[]) {
    // insert code here...
    srand( static_cast<unsigned int>(time(0)));
    std::cout << "Hello, World!\n";
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    
    GLFWwindow* window = glfwCreateWindow(width, height, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    
    // tell GLFW to capture our mouse
//    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    
    Shader shaderProgram("default.vert", "default.frag");
    
    glEnable(GL_DEPTH_TEST);
    
    float grid[] = {
         0.0f,  0.0f,  0.0f,
         0.0f,  0.5f,  0.0f,
         0.5f,  0.5f,  0.0f,
         0.5f,  0.0f,  0.0f,
    };
    VAO vao;
    vao.Bind();
    
    VBO vbo(grid, sizeof(grid));
    
    vao.LinkAttrib(vbo, 0, 3, GL_FLOAT, 3*sizeof(float), (void*)0);
    
    vao.Unbind();
    vbo.Unbind();
    
    // world space positions of cube
    std::vector<glm::vec3> gridPositions;
    int slices = 50;
    for(int j=0; j<slices; ++j) {
        for(int i=0; i<slices; ++i) {
          float row1 =  i * 0.5;
          float row2 =  j * 0.5;
          gridPositions.push_back(glm::vec3(row1, row2, 0));

        }
      }
    
    
    Robot robot;
    Controller control;
    initialize_robot(robot);
    initialize_controller(control);
    
    cout<< robot.masses.size() << endl;
    cout<< robot.springs.size() << endl;
    cout<< robot.all_cubes.size() << endl;
    cout<<robot.all_cubes.back().massIDs.size() << endl;
    cout<<robot.all_cubes.back().springIDs.size() << endl;
    
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    for (int m=0; m<robot.masses.size(); m++){
        x_center += robot.masses[m].position[0];
        y_center += robot.masses[m].position[1];
        z_center += robot.masses[m].position[2];
    }
    
    x_center = x_center/robot.masses.size();
    y_center = y_center/robot.masses.size();
    z_center = z_center/robot.masses.size();
    
    control.start = {x_center, y_center, z_center};
    
    cout << "Center = ";
    cout << x_center;
    cout << ", ";
    cout << y_center;
    cout << ", ";
    cout << z_center << endl;
    
    int iterations = 0;
    
    float x0 = robot.masses[0].position[0];
    float y0 = robot.masses[0].position[1];
    float z0 = robot.masses[0].position[2];
    float x1 = robot.masses[1].position[0];
    float y1 = robot.masses[1].position[1];
    float z1 = robot.masses[1].position[2];
    float x2 = robot.masses[2].position[0];
    float y2 = robot.masses[2].position[1];
    float z2 = robot.masses[2].position[2];
    float x3 = robot.masses[3].position[0];
    float y3 = robot.masses[3].position[1];
    float z3 = robot.masses[3].position[2];
    float x4 = robot.masses[4].position[0];
    float y4 = robot.masses[4].position[1];
    float z4 = robot.masses[4].position[2];
    float x5 = robot.masses[5].position[0];
    float y5 = robot.masses[5].position[1];
    float z5 = robot.masses[5].position[2];
    float x6 = robot.masses[6].position[0];
    float y6 = robot.masses[6].position[1];
    float z6 = robot.masses[6].position[2];
    float x7 = robot.masses[7].position[0];
    float y7 = robot.masses[7].position[1];
    float z7 = robot.masses[7].position[2];
    
    vector<float> PE; //total potential energy of the system
    vector<float> KE; //total kinetic energy of the system
    vector<float> TE; //total energy of the system
    
    // render loop
    while(!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = (currentFrame - lastFrame);
        lastFrame = currentFrame;
        
        processInput(window);
        
        // render
        // ------
        glClearColor(0.1f, 0.3f, 0.4f, 1.0f);
//        glClearColor(0.07f, 0.13f, 0.17f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        shaderProgram.Activate();
        
        // pass projection matrix to shader (note that in this case it could change every frame)
        glm::mat4 proj = glm::perspective(glm::radians(fov), (float)width / (float)height, 0.1f, 100.0f);
        glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        int viewLoc = glGetUniformLocation(shaderProgram.ID, "view");
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        int projLoc = glGetUniformLocation(shaderProgram.ID, "proj");
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(proj));
        
        // render the grid
        //-------------------------------------
        for (unsigned int i = 0; i < 2500; i++){
            vao.Bind();
            glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
            model = glm::translate(model, gridPositions[i]);
            float angle = 20.0f * 0;
            model = glm::rotate(model, glm::radians(angle), glm::vec3(1.0f, 0.3f, 0.5f));
            int modelLoc = glGetUniformLocation(shaderProgram.ID, "model");
            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            glDrawArrays(GL_LINE_LOOP, 0, 4);
        }
        //-------------------------------------
        
        //Update the forces, acceleration, velocity, and position
        //-------------------------------------
        for (int k=0; k<100; k++){
            T = T + dt; //update time that has passed
            if (breathing) {
                update_breathing(robot, control);
            }

            update_forces(robot);
            update_pos_vel_acc(robot);

            reset_forces(robot);

        }
        //-------------------------------------
        
        
        //Update the position on the actual simulator only after every 50 simulations
        //-------------------------------------
        for (int j=0; j<robot.all_cubes.size(); j++){
            if (iterations % 1 == 0){
                x0 = robot.all_cubes[j].masses[0].position[0];
                y0 = robot.all_cubes[j].masses[0].position[1];
                z0 = robot.all_cubes[j].masses[0].position[2];
                x1 = robot.all_cubes[j].masses[1].position[0];
                y1 = robot.all_cubes[j].masses[1].position[1];
                z1 = robot.all_cubes[j].masses[1].position[2];
                x2 = robot.all_cubes[j].masses[2].position[0];
                y2 = robot.all_cubes[j].masses[2].position[1];
                z2 = robot.all_cubes[j].masses[2].position[2];
                x3 = robot.all_cubes[j].masses[3].position[0];
                y3 = robot.all_cubes[j].masses[3].position[1];
                z3 = robot.all_cubes[j].masses[3].position[2];
                x4 = robot.all_cubes[j].masses[4].position[0];
                y4 = robot.all_cubes[j].masses[4].position[1];
                z4 = robot.all_cubes[j].masses[4].position[2];
                x5 = robot.all_cubes[j].masses[5].position[0];
                y5 = robot.all_cubes[j].masses[5].position[1];
                z5 = robot.all_cubes[j].masses[5].position[2];
                x6 = robot.all_cubes[j].masses[6].position[0];
                y6 = robot.all_cubes[j].masses[6].position[1];
                z6 = robot.all_cubes[j].masses[6].position[2];
                x7 = robot.all_cubes[j].masses[7].position[0];
                y7 = robot.all_cubes[j].masses[7].position[1];
                z7 = robot.all_cubes[j].masses[7].position[2];
            }
            //-------------------------------------
            
            // update the cube
            //-------------------------------------
            GLfloat vertices[] =
            {
                x0, y0, z0,      0.83f, 0.70f, 0.44f,
                x1, y1, z1,      0.83f, 0.70f, 0.44f,
                x2, y2, z2,      0.83f, 0.70f, 0.44f,
                x3, y3, z3,      0.83f, 0.70f, 0.44f,
                x4, y4, z4,      0.92f, 0.86f, 0.76f,
                x5, y5, z5,      0.92f, 0.86f, 0.76f,
                x6, y6, z6,      0.92f, 0.86f, 0.76f,
                x7, y7, z7,      0.92f, 0.86f, 0.76f,
            };
//            GLfloat vertices[] =
//            {
//                x0, y0, z0,      0.83f, 0.70f, 0.44f,
//                x1, y1, z1,      0.83f, 0.70f, 0.44f,
//                x2, y2, z2,      0.83f, 0.70f, 0.44f,
//                x3, y3, z3,      0.83f, 0.70f, 0.44f,
//                x4, y4, z4,      0.83f, 0.70f, 0.44f,
//                x5, y5, z5,      0.83f, 0.70f, 0.44f,
//                x6, y6, z6,      0.83f, 0.70f, 0.44f,
//                x7, y7, z7,      0.83f, 0.70f, 0.44f,
//            };
            
//            GLuint indices[] =
//            {
//                0, 1, 2,
//                0, 2, 3,
//                0, 1, 5,
//                0, 4, 5,
//                1, 2, 6,
//                1, 5, 6,
//                2, 3, 7,
//                2, 6, 7,
//                4, 5, 6,
//                4, 6, 7,
//                0, 3, 7,
//                0, 4, 7
//            };
            GLuint indices[] =
            {
                0, 1,
                1, 2,
                2, 3,
                3, 0,
                0, 2,
                1, 3,
                0, 4,
                1, 5,
                0, 5,
                1, 4,
                2, 6,
                1, 6,
                2, 5,
                3, 7,
                2, 7,
                3, 6,
                4, 5,
                5, 6,
                6, 7,
                4, 7,
                4, 6,
                5, 7,
                0, 6,
                2, 4,
                3, 5,
                1, 7,
                0, 7,
                3, 4
            };
            
            //-------------------------------------
            
            glm::vec3 cubePositions[] = {
                glm::vec3( 10.0f, 10.0f, 0.0f),
            };
            
            VAO VAO1;
            VAO1.Bind();
            
            VBO VBO1(vertices, sizeof(vertices));
            EBO EBO1(indices, sizeof(indices));
    //        GLuint indexBuffer;
    //        glGenBuffers(1, &indexBuffer);
    //        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
    //        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size(), indices.data(), GL_STATIC_DRAW);
            
            VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 6*sizeof(float), (void*)0);
            VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 6*sizeof(float), (void*)(3*sizeof(float)));
            VAO1.Unbind();
            VBO1.Unbind();
            EBO1.Unbind();
            
            VAO1.Bind();
            glm::mat4 model = glm::mat4(1.0f);
            model = glm::translate(model, cubePositions[0]);
            float angle = 20.0f * 0;
            model = glm::rotate(model, glm::radians(angle), glm::vec3(1.0f, 0.3f, 0.5f));
            int modelLoc = glGetUniformLocation(shaderProgram.ID, "model");
            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            glDrawArrays(GL_LINE_LOOP, 0, 4);
            glDrawElements(GL_LINES, sizeof(indices)/sizeof(int), GL_UNSIGNED_INT, 0);
//            glDrawElements(GL_TRIANGLES, sizeof(indices)/sizeof(int), GL_UNSIGNED_INT, 0);

            
            VAO1.Delete();
            VBO1.Delete();
            EBO1.Delete();
        }
        
        if (iterations==300){
            float x_center_final = 0;
            float y_center_final = 0;
            float z_center_final = 0;
            for (int m=0; m<robot.masses.size(); m++){
                x_center_final += robot.masses[m].position[0];
                y_center_final += robot.masses[m].position[1];
                z_center_final += robot.masses[m].position[2];
            }

            x_center_final = x_center_final/robot.masses.size();
            y_center_final = y_center_final/robot.masses.size();
            z_center_final = z_center_final/robot.masses.size();
            control.end = {x_center_final, y_center_final, z_center_final};

            float displacement = sqrt(pow(control.end[0]-control.start[0], 2) + pow(control.end[1]-control.start[1], 2));
            cout << displacement << endl;
        }
//        reset_forces(robot);
        iterations += 1;
//        cout << iterations << endl;
        
        glfwSwapBuffers(window);
        glfwPollEvents();
        
    }
    
    shaderProgram.Delete();

    glfwTerminate();
    
    return 0;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void update_pos_vel_acc(Robot &robot){
    
    for (int i=0; i<robot.masses.size(); i++){
        float acc_x = robot.masses[i].forces[0]/robot.masses[i].mass;
        float acc_y = robot.masses[i].forces[1]/robot.masses[i].mass;
        float acc_z = robot.masses[i].forces[2]/robot.masses[i].mass;

        robot.masses[i].acceleration[0] = acc_x;
        robot.masses[i].acceleration[1] = acc_y;
        robot.masses[i].acceleration[2] = acc_z;
        
        float vel_x = acc_x*dt + robot.masses[i].velocity[0];
        float vel_y = acc_y*dt + robot.masses[i].velocity[1];
        float vel_z = acc_z*dt + robot.masses[i].velocity[2];
        
        
        robot.masses[i].velocity[0] = vel_x*b;
        robot.masses[i].velocity[1] = vel_y*b;
        robot.masses[i].velocity[2] = vel_z*b;
        
        float pos_x = (vel_x*dt) + robot.masses[i].position[0];
        float pos_y = (vel_y*dt) + robot.masses[i].position[1];
        float pos_z = (vel_z*dt) + robot.masses[i].position[2];
        
        robot.masses[i].position[0] = pos_x;
        robot.masses[i].position[1] = pos_y;
        robot.masses[i].position[2] = pos_z;
    }
    
    for (int j=0; j<robot.all_cubes.size(); j++){
        int ind0 = robot.all_cubes[j].massIDs[0];
        int ind1 = robot.all_cubes[j].massIDs[1];
        int ind2 = robot.all_cubes[j].massIDs[2];
        int ind3 = robot.all_cubes[j].massIDs[3];
        int ind4 = robot.all_cubes[j].massIDs[4];
        int ind5 = robot.all_cubes[j].massIDs[5];
        int ind6 = robot.all_cubes[j].massIDs[6];
        int ind7 = robot.all_cubes[j].massIDs[7];
        
        for (int k=0; k<8; k++){
            if (robot.all_cubes[j].masses[k].ID == ind0){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind0].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind0].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind0].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind1){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind1].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind1].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind1].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind2){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind2].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind2].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind2].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind3){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind3].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind3].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind3].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind4){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind4].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind4].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind4].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind5){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind5].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind5].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind5].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind6){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind6].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind6].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind6].position[2];
            }
            else if (robot.all_cubes[j].masses[k].ID == ind7){
                robot.all_cubes[j].masses[k].position[0] = robot.masses[ind7].position[0];
                robot.all_cubes[j].masses[k].position[1] = robot.masses[ind7].position[1];
                robot.all_cubes[j].masses[k].position[2] = robot.masses[ind7].position[2];
            }
        }
//
//        robot.all_cubes[j].masses[0].position[0] = robot.masses[ind0].position[0];
//        robot.all_cubes[j].masses[0].position[1] = robot.masses[ind0].position[1];
//        robot.all_cubes[j].masses[0].position[2] = robot.masses[ind0].position[2];
//
//        robot.all_cubes[j].masses[1].position[0] = robot.masses[ind1].position[0];
//        robot.all_cubes[j].masses[1].position[1] = robot.masses[ind1].position[1];
//        robot.all_cubes[j].masses[1].position[2] = robot.masses[ind1].position[2];
//
//        robot.all_cubes[j].masses[2].position[0] = robot.masses[ind2].position[0];
//        robot.all_cubes[j].masses[2].position[1] = robot.masses[ind2].position[1];
//        robot.all_cubes[j].masses[2].position[2] = robot.masses[ind2].position[2];
//
//        robot.all_cubes[j].masses[3].position[0] = robot.masses[ind3].position[0];
//        robot.all_cubes[j].masses[3].position[1] = robot.masses[ind3].position[1];
//        robot.all_cubes[j].masses[3].position[2] = robot.masses[ind3].position[2];
//
//        robot.all_cubes[j].masses[4].position[0] = robot.masses[ind4].position[0];
//        robot.all_cubes[j].masses[4].position[1] = robot.masses[ind4].position[1];
//        robot.all_cubes[j].masses[4].position[2] = robot.masses[ind4].position[2];
//
//        robot.all_cubes[j].masses[5].position[0] = robot.masses[ind5].position[0];
//        robot.all_cubes[j].masses[5].position[1] = robot.masses[ind5].position[1];
//        robot.all_cubes[j].masses[5].position[2] = robot.masses[ind5].position[2];
//
//        robot.all_cubes[j].masses[6].position[0] = robot.masses[ind6].position[0];
//        robot.all_cubes[j].masses[6].position[1] = robot.masses[ind6].position[1];
//        robot.all_cubes[j].masses[6].position[2] = robot.masses[ind6].position[2];
//
//        robot.all_cubes[j].masses[7].position[0] = robot.masses[ind7].position[0];
//        robot.all_cubes[j].masses[7].position[1] = robot.masses[ind7].position[1];
//        robot.all_cubes[j].masses[7].position[2] = robot.masses[ind7].position[2];
    }
}

void reset_forces(Robot &robot){
    for(int i=0; i<robot.masses.size(); i++){
        robot.masses[i].forces = {0.0f, 0.0f, 0.0f};
    }
}

void update_forces(Robot &robot){
    
    for (int i=0; i<robot.springs.size(); i++){

        int p0 = robot.springs[i].m0;
        int p1 = robot.springs[i].m1;

        vector<float> pos0 = robot.masses[p0].position;
        vector<float> pos1 = robot.masses[p1].position;

        float spring_length = sqrt(pow(pos1[0]-pos0[0], 2) + pow(pos1[1]-pos0[1], 2) + pow(pos1[2]-pos0[2], 2));

        robot.springs[i].L = spring_length;
        float force = -robot.springs[i].k*(spring_length-robot.springs[i].L0);

        float x_univ = (pos0[0]-pos1[0])/spring_length;
        float y_univ = (pos0[1]-pos1[1])/spring_length;
        float z_univ = (pos0[2]-pos1[2])/spring_length;
        vector<float> force_unit_dir_2_1 = {x_univ,y_univ,z_univ};
        vector<float> force_unit_dir_1_2 = {-x_univ,-y_univ,-z_univ};

        for (int n = 0; n < 3; n++) {
            robot.masses[p0].forces[n] =  robot.masses[p0].forces[n] + force * force_unit_dir_2_1[n];
            robot.masses[p1].forces[n] =  robot.masses[p1].forces[n] + force * force_unit_dir_1_2[n];
        }
    }
    
    for (int j=0; j<robot.masses.size(); j++){
        robot.masses[j].forces[2] = robot.masses[j].forces[2] + robot.masses[j].mass*g;
        
        if (robot.masses[j].position[2] < 0){
            robot.masses[j].forces[2] = -robot.masses[j].position[2]*1000000.0f;
        }
        
        float F_n = robot.masses[j].mass*g;

        float F_h = sqrt(pow(robot.masses[j].forces[0], 2) + pow(robot.masses[j].forces[1], 2));


        if (F_n < 0){
            if (F_h < -F_n*mu_s){
                robot.masses[j].forces[0] = 0;
                robot.masses[j].forces[1] = 0;
            }
            if (F_h >= -F_n*mu_s){
                if (robot.masses[j].forces[0] > 0){
                    robot.masses[j].forces[0] = robot.masses[j].forces[0] + mu_k*F_n;
                }
                else{
                    robot.masses[j].forces[0] = robot.masses[j].forces[0] - mu_k*F_n;
                }
                if (robot.masses[j].forces[1] > 0){
                    robot.masses[j].forces[1] = robot.masses[j].forces[1] + mu_k*F_n;
                }
                else{
                    robot.masses[j].forces[1] = robot.masses[j].forces[1] - mu_k*F_n;
                }
            }
        }
    }
}

void update_breathing(Robot &robot, Controller &control){
    for (int i=0; i<robot.all_cubes.size(); i++){
        int ind0 = robot.all_cubes[i].springIDs[0];
        int ind1 = robot.all_cubes[i].springIDs[1];
        int ind2 = robot.all_cubes[i].springIDs[2];
        int ind3 = robot.all_cubes[i].springIDs[3];
        int ind4 = robot.all_cubes[i].springIDs[4];
        int ind5 = robot.all_cubes[i].springIDs[5];
        int ind6 = robot.all_cubes[i].springIDs[6];
        int ind7 = robot.all_cubes[i].springIDs[7];
        int ind8 = robot.all_cubes[i].springIDs[8];
        int ind9 = robot.all_cubes[i].springIDs[9];
        int ind10 = robot.all_cubes[i].springIDs[10];
        int ind11 = robot.all_cubes[i].springIDs[11];
        int ind12 = robot.all_cubes[i].springIDs[12];
        int ind13 = robot.all_cubes[i].springIDs[13];
        int ind14 = robot.all_cubes[i].springIDs[14];
        int ind15 = robot.all_cubes[i].springIDs[15];
        int ind16 = robot.all_cubes[i].springIDs[16];
        int ind17 = robot.all_cubes[i].springIDs[17];
        int ind18 = robot.all_cubes[i].springIDs[18];
        int ind19 = robot.all_cubes[i].springIDs[19];
        int ind20 = robot.all_cubes[i].springIDs[20];
        int ind21 = robot.all_cubes[i].springIDs[21];
        int ind22 = robot.all_cubes[i].springIDs[22];
        int ind23 = robot.all_cubes[i].springIDs[23];
        int ind24 = robot.all_cubes[i].springIDs[24];
        int ind25 = robot.all_cubes[i].springIDs[25];
        int ind26 = robot.all_cubes[i].springIDs[26];
        int ind27 = robot.all_cubes[i].springIDs[27];

        float k = control.motor[i].k;
        float a = control.motor[i].a;
        float w = control.motor[i].w;
        float c = control.motor[i].c;

        robot.springs[ind0].L0 = robot.all_cubes[i].springs[0].original_L0 + a*sin(w*T+c);
        robot.springs[ind1].L0 = robot.all_cubes[i].springs[1].original_L0 + a*sin(w*T+c);
        robot.springs[ind2].L0 = robot.all_cubes[i].springs[2].original_L0 + a*sin(w*T+c);
        robot.springs[ind3].L0 = robot.all_cubes[i].springs[3].original_L0 + a*sin(w*T+c);
        robot.springs[ind4].L0 = robot.all_cubes[i].springs[4].original_L0 + a*sin(w*T+c);
        robot.springs[ind5].L0 = robot.all_cubes[i].springs[5].original_L0 + a*sin(w*T+c);
        robot.springs[ind6].L0 = robot.all_cubes[i].springs[6].original_L0 + a*sin(w*T+c);
        robot.springs[ind7].L0 = robot.all_cubes[i].springs[7].original_L0 + a*sin(w*T+c);
        robot.springs[ind8].L0 = robot.all_cubes[i].springs[8].original_L0 + a*sin(w*T+c);
        robot.springs[ind9].L0 = robot.all_cubes[i].springs[9].original_L0 + a*sin(w*T+c);
        robot.springs[ind10].L0 = robot.all_cubes[i].springs[10].original_L0 + a*sin(w*T+c);
        robot.springs[ind11].L0 = robot.all_cubes[i].springs[11].original_L0 + a*sin(w*T+c);
        robot.springs[ind12].L0 = robot.all_cubes[i].springs[12].original_L0 + a*sin(w*T+c);
        robot.springs[ind13].L0 = robot.all_cubes[i].springs[13].original_L0 + a*sin(w*T+c);
        robot.springs[ind14].L0 = robot.all_cubes[i].springs[14].original_L0 + a*sin(w*T+c);
        robot.springs[ind15].L0 = robot.all_cubes[i].springs[15].original_L0 + a*sin(w*T+c);
        robot.springs[ind16].L0 = robot.all_cubes[i].springs[16].original_L0 + a*sin(w*T+c);
        robot.springs[ind17].L0 = robot.all_cubes[i].springs[17].original_L0 + a*sin(w*T+c);
        robot.springs[ind18].L0 = robot.all_cubes[i].springs[18].original_L0 + a*sin(w*T+c);
        robot.springs[ind19].L0 = robot.all_cubes[i].springs[19].original_L0 + a*sin(w*T+c);
        robot.springs[ind20].L0 = robot.all_cubes[i].springs[20].original_L0 + a*sin(w*T+c);
        robot.springs[ind21].L0 = robot.all_cubes[i].springs[21].original_L0 + a*sin(w*T+c);
        robot.springs[ind22].L0 = robot.all_cubes[i].springs[22].original_L0 + a*sin(w*T+c);
        robot.springs[ind23].L0 = robot.all_cubes[i].springs[23].original_L0 + a*sin(w*T+c);
        robot.springs[ind24].L0 = robot.all_cubes[i].springs[24].original_L0 + a*sin(w*T+c);
        robot.springs[ind25].L0 = robot.all_cubes[i].springs[25].original_L0 + a*sin(w*T+c);
        robot.springs[ind26].L0 = robot.all_cubes[i].springs[26].original_L0 + a*sin(w*T+c);
        robot.springs[ind27].L0 = robot.all_cubes[i].springs[27].original_L0 + a*sin(w*T+c);

        robot.springs[ind0].k = k;
        robot.springs[ind1].k = k;
        robot.springs[ind2].k = k;
        robot.springs[ind3].k = k;
        robot.springs[ind4].k = k;
        robot.springs[ind5].k = k;
        robot.springs[ind6].k = k;
        robot.springs[ind7].k = k;
        robot.springs[ind8].k = k;
        robot.springs[ind9].k = k;
        robot.springs[ind10].k = k;
        robot.springs[ind11].k = k;
        robot.springs[ind12].k = k;
        robot.springs[ind13].k = k;
        robot.springs[ind14].k = k;
        robot.springs[ind15].k = k;
        robot.springs[ind16].k = k;
        robot.springs[ind17].k = k;
        robot.springs[ind18].k = k;
        robot.springs[ind19].k = k;
        robot.springs[ind20].k = k;
        robot.springs[ind21].k = k;
        robot.springs[ind22].k = k;
        robot.springs[ind23].k = k;
        robot.springs[ind24].k = k;
        robot.springs[ind25].k = k;
        robot.springs[ind26].k = k;
        robot.springs[ind27].k = k;
    }
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS){
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
        
        if(firstClick){
            glfwSetCursorPos(window, (width/2), (height/2));
            lastX = xpos;
            lastY = ypos;
            firstClick = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
        lastX = xpos;
        lastY = ypos;

        float sensitivity = 0.1f; // change this value to your liking
        xoffset *= sensitivity;
        yoffset *= sensitivity;

        yaw += xoffset;
        pitch += yoffset;

        // make sure that when pitch is out of bounds, screen doesn't get flipped
        if (pitch > 89.0f)
            pitch = 89.0f;
        if (pitch < -89.0f)
            pitch = -89.0f;

        glm::vec3 front;
        front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
        front.y = sin(glm::radians(pitch));
        front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
        cameraFront = glm::normalize(front);
    }
    else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE){
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        firstClick = true;
    }
    
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    fov -= (float)yoffset;
    if (fov < 1.0f)
        fov = 1.0f;
    if (fov > 45.0f)
        fov = 45.0f;
}

void initialize_robot(Robot &robot){
    vector<PointMass> masses; //initializes the vector of masses that make up the robot
    vector<Spring> springs; //initializes the vector of springs that make up the robot
    vector<int> cubes;
    vector<Cube> all_cubes; //initializes all the cubes that will make up this robot
    vector<int> available_cubes;
    for (int i=0; i<10; i++){
        Cube cube; //define a cube
        initialize_cube(cube); //initialize the cube
        if (i==0){
            //for the first cube, you can add everything
            for (int j=0; j<28; j++){
                cube.springs[j].ID = j;
                cube.springIDs.push_back(j);
                springs.push_back(cube.springs[j]);
            }
            for (int k=0; k<8; k++){
                cube.masses[k].ID = k;
                cube.massIDs.push_back(k);
                masses.push_back(cube.masses[k]);
            }
            available_cubes.push_back(i);
            
        }
        else{
            int cube1 = rand() % available_cubes.size();
//            cout << all_cubes[available_cubes[cube1]].free_faces.size() << endl;
            cube1 = available_cubes[cube1];
            int face_1 = rand() % all_cubes[cube1].free_faces.size();
            int cube1_face1 = all_cubes[cube1].free_faces[face_1];
//            int cube1_face1 = 5;
            int face_2;
            vector<int> map1;
            vector<int> map2;
            vector<int> masses_left;
            vector<int> springs_left;
            
            for (int s=0; s<8; s++){
                masses_left.push_back(s);
            }
            
            for (int v=0; v<28; v++){
                springs_left.push_back(v);
            }
            
            if (cube1_face1 == 0){
                face_2 = 5;
                
                map1 = face0;
                map2 = face5;
            }
            else if (cube1_face1 == 5){
                face_2 = 0;
                
                map1 = face5;
                map2 = face0;
            }
            else if (cube1_face1 == 1){
                face_2 = 3;
                
                map1 = face1;
                map2 = face3;
            }
            else if (cube1_face1 == 3){
                face_2 = 1;
                
                map1 = face3;
                map2 = face1;
            }
            else if (cube1_face1 == 2){
                face_2 = 4;
                
                map1 = face2;
                map2 = face4;
            }
            else{
                face_2 = 2;
                
                map1 = face4;
                map2 = face2;
            }
            
            int itr = find(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), cube1_face1)-all_cubes[cube1].free_faces.begin();
            int itr2 = find(cube.free_faces.begin(), cube.free_faces.end(), face_2)-cube.free_faces.begin();
            
            all_cubes[cube1].free_faces.erase(all_cubes[cube1].free_faces.begin()+itr);
            cube.free_faces.erase(cube.free_faces.begin()+itr2);
//            remove(all_cubes[cube1].free_faces.begin(), all_cubes[cube1].free_faces.end(), all_cubes[cube1].free_faces[face_1]);
//            remove(cube.free_faces.begin(), cube.free_faces.end(), cube.free_faces[face_2]);
            
            float cube1_z0 = all_cubes[cube1].masses[0].position[2];
            
            if (cube1_face1 == 0 && cube1_z0 == 0){
                cout << "Need to shift the robot up" << endl;
                float x_disp = all_cubes[cube1].masses[map1[0]].position[0]-cube.masses[map2[0]].position[0]; //x displacement
                float y_disp = all_cubes[cube1].masses[map1[0]].position[1]-cube.masses[map2[0]].position[1]; //y displacement
                float z_disp = all_cubes[cube1].masses[map1[0]].position[2]-cube.masses[map2[0]].position[2]; //z displacement
                
                for (int m=0; m<all_cubes.size(); m++){
                    for (int n=0; n<8; n++){
                        //shift cube 2 over
                        all_cubes[m].masses[n].position[0] -= x_disp;
                        all_cubes[m].masses[n].position[1] -= y_disp;
                        all_cubes[m].masses[n].position[2] -= z_disp;
                        
                        masses[all_cubes[m].masses[n].ID].position[0] = all_cubes[m].masses[n].position[0];
                        masses[all_cubes[m].masses[n].ID].position[1] = all_cubes[m].masses[n].position[1];
                        masses[all_cubes[m].masses[n].ID].position[2] = all_cubes[m].masses[n].position[2];
                    }
                    all_cubes[m].center[0] -= x_disp;
                    all_cubes[m].center[1] -= y_disp;
                    all_cubes[m].center[2] -= z_disp;
                }
            }
            else{
                //find where the second cube needs to join the first cube
                float x_disp = cube.masses[map2[0]].position[0]-all_cubes[cube1].masses[map1[0]].position[0]; //x displacement
                float y_disp = cube.masses[map2[0]].position[1]-all_cubes[cube1].masses[map1[0]].position[1]; //y displacement
                float z_disp = cube.masses[map2[0]].position[2]-all_cubes[cube1].masses[map1[0]].position[2]; //z displacement
                
                for (int u=0; u<8; u++){
                    //shift cube 2 over
                    cube.masses[u].position[0] -= x_disp;
                    cube.masses[u].position[1] -= y_disp;
                    cube.masses[u].position[2] -= z_disp;
                }
                
                cube.center[0] -= x_disp;
                cube.center[1] -= y_disp;
                cube.center[2] -= z_disp;
            }
            cout << "Face 2 = ";
            cout << face_2 << endl;
            
            fuse_faces(all_cubes[cube1], cube, cube1, i, masses, springs, cube1_face1, face_2, masses_left, springs_left);
            
            for (int q=0; q<all_cubes.size(); q++){
                if (all_cubes[q].center[0]-cube.center[0] == 0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                    cout << "Also a cube to the right" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 2)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 4)-cube.free_faces.begin();
                    
                    cout << "Here: ";
                    cout << itr3 << endl;
                    cout << all_cubes[q].free_faces[itr3] << endl;
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 2, 4, masses_left, springs_left);
                }
                else if (all_cubes[q].center[0]-cube.center[0] == -0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                    cout << "Also a cube to the left" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 4)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 2)-cube.free_faces.begin();
                    
                    cout << "Here: ";
                    cout << itr3 << endl;
                    cout << all_cubes[q].free_faces[itr3] << endl;
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 4, 2, masses_left, springs_left);
                }
                else if (all_cubes[q].center[1]-cube.center[1] == 0.5 && all_cubes[q].center[0]-cube.center[0] == 0 && all_cubes[q].center[2]-cube.center[2] == 0 && q != cube1){
                    cout << "Also a cube in front" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 1)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 3)-cube.free_faces.begin();
                    
                    cout << "Here: ";
                    cout << itr3 << endl;
                    cout << all_cubes[q].free_faces[itr3] << endl;
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 1, 3, masses_left, springs_left);
                }
                else if (all_cubes[q].center[1]-cube.center[1] == -0.5 && all_cubes[q].center[2]-cube.center[2] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                    cout << "Also a cube in back" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 3)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 1)-cube.free_faces.begin();
                    
                    cout << "Here: ";
                    cout << itr3 << endl;
                    cout << all_cubes[q].free_faces[itr3] << endl;
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 3, 1, masses_left, springs_left);
                }
                else if (all_cubes[q].center[2]-cube.center[2] == 0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                    cout << "Also a cube on top" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 0)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 5)-cube.free_faces.begin();
                    
                    cout << "Here: ";
                    cout << itr3 << endl;
                    cout << all_cubes[q].free_faces[itr3] << endl;
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 0, 5, masses_left, springs_left);
                }
                else if (all_cubes[q].center[2]-cube.center[2] == -0.5 && all_cubes[q].center[1]-cube.center[1] == 0 && all_cubes[q].center[0]-cube.center[0] == 0 && q != cube1){
                    cout << "Also a cube on bottom" << endl;
                    
                    int itr3 = find(all_cubes[q].free_faces.begin(), all_cubes[q].free_faces.end(), 5)-all_cubes[q].free_faces.begin();
                    int itr4 = find(cube.free_faces.begin(), cube.free_faces.end(), 0)-cube.free_faces.begin();
                    
                    cout << "Here: ";
                    cout << itr3 << endl;
                    cout << all_cubes[q].free_faces[itr3] << endl;
                    
                    all_cubes[q].free_faces.erase(all_cubes[q].free_faces.begin()+itr3);
                    cube.free_faces.erase(cube.free_faces.begin()+itr4);
                    
                    fuse_faces(all_cubes[q], cube, q, i, masses, springs, 5, 0, masses_left, springs_left);
                }
            }
            
            cout << "Masses left = ";
            cout << masses_left.size() << endl;
            
            for (int j=0; j<masses_left.size(); j++){
                // if the vertex is not part of face 2 then you can add it to the big vector of masses and make the ID the index of where it is in the big vector of masses
                cube.masses[masses_left[j]].ID = masses.size();
                cube.massIDs.push_back(masses.size());
                masses.push_back(cube.masses[masses_left[j]]);
                
                cout << "j = ";
                cout << j << endl;
                cout << "Mass = ";
                cout << masses_left[j] << endl;
                cout << "ID = ";
                cout << cube.masses[masses_left[j]].ID << endl;
                cout << "--------" << endl;
                
            }
            
            cout << "Springs left = ";
            cout << springs_left.size() << endl;
            
            for (int k=0; k<springs_left.size(); k++){
                int p0 = cube.springs[springs_left[k]].m0;
                int p1 = cube.springs[springs_left[k]].m1;
                
                cout << "k = ";
                cout << k << endl;
                cout << "Spring = ";
                cout << springs_left[k] << endl;
                cout << "p0 = ";
                cout << p0 << endl;
                cout << "p1 = ";
                cout << p1 << endl;
                cout << "--------" << endl;
                cube.springs[springs_left[k]].m0 = cube.masses[p0].ID;
                cube.springs[springs_left[k]].m1 = cube.masses[p1].ID;
                cube.springs[springs_left[k]].ID = springs.size();
                cube.springIDs.push_back(springs.size());
                springs.push_back(cube.springs[springs_left[k]]);
                
            }
            
            if (cube.free_faces.size() < 1){
                cout << "Maximized fused faces on this cube" << endl;
            }
            else{
                available_cubes.push_back(i);
            }
            if (all_cubes[cube1].free_faces.size() < 1){
                cout << "Maximized fused faces on this cube" << endl;
                int itr5 = find(available_cubes.begin(), available_cubes.end(), cube1)-available_cubes.begin();
//                remove(available_cubes.begin(), available_cubes.end(), available_cubes[cube1]);
                available_cubes.erase(available_cubes.begin()+itr5);
            }
            
        }
        
        for (int t=0; t<8; t++){
            cout << "MASSES" << endl;
            cout << t;
            cout << ", ";
            cout << cube.masses[t].ID << endl;
            cout << "--------" << endl;
        }
        
        cubes.push_back(i);
        all_cubes.push_back(cube);
    }
    robot.masses = masses;
    robot.springs = springs;
    robot.all_cubes = all_cubes;
    robot.available_cubes = available_cubes;
    cout<< "Hello" << endl;
    
    for (int j=0; j<robot.springs.size(); j++){
        cout << j;
        cout << ", ";
        cout << robot.springs[j].m0;
        cout << ", ";
        cout << robot.springs[j].m1 << endl;
    }
}

void fuse_faces(Cube &cube1, Cube &cube2, int cube1_index, int cube2_index, vector<PointMass> &masses, vector<Spring> &springs, int combine1, int combine2, vector<int> &masses_left, vector<int> &springs_left){
    
    vector<int> map1;
    vector<int> map2;
    vector<int> map1_springs;
    vector<int> map2_springs;
    
    if (combine1 == 0){
        map1 = face0;
        map2 = face5;
        
        map1_springs = face0_springs;
        map2_springs = face5_springs;
    }
    else if (combine1 == 5){
        map1 = face5;
        map2 = face0;
        
        map1_springs = face5_springs;
        map2_springs = face0_springs;
    }
    else if (combine1 == 1){
        map1 = face1;
        map2 = face3;
        
        map1_springs = face1_springs;
        map2_springs = face3_springs;
    }
    else if (combine1 == 3){
        map1 = face3;
        map2 = face1;
        
        map1_springs = face3_springs;
        map2_springs = face1_springs;
    }
    else if (combine1 == 2){
        map1 = face2;
        map2 = face4;
        
        map1_springs = face2_springs;
        map2_springs = face4_springs;
    }
    else{
        map1 = face4;
        map2 = face2;
        
        map1_springs = face4_springs;
        map2_springs = face2_springs;
    }
    
    cube1.joinedCubes.push_back(cube2_index);
    cube1.joinedFaces.push_back(combine1);
    cube1.otherFaces.push_back(combine2);
    
    cube2.joinedCubes.push_back(cube1_index);
    cube2.joinedFaces.push_back(combine2);
    cube2.otherFaces.push_back(combine1);
    
    //joining cube 2 on the right face of the first cube; this means the left face of cube 2 and the right face of cube 1 will be joined
    for (int j=0; j<map2.size(); j++){
        if (find(cube2.massIDs.begin(), cube2.massIDs.end(), cube1.masses[map1[j]].ID) == cube2.massIDs.end()){
            cube2.masses[map2[j]].ID = cube1.masses[map1[j]].ID; //set the mass ID to its position in the masses vector of the robot
            cube2.massIDs.push_back(cube1.masses[map1[j]].ID); //add the mass IDs to the list of masses that correspond to cube2
        }
        
        if (find(masses_left.begin(), masses_left.end(), map2[j]) != masses_left.end()){
//            remove(masses_left.begin(), masses_left.end(), map2[j]);
            int itr = find(masses_left.begin(), masses_left.end(), map2[j])-masses_left.begin();
            masses_left.erase(masses_left.begin()+itr);
        }
    }
    
    for (int k=0; k<map2_springs.size(); k++){
        int p0 = cube2.springs[map2_springs[k]].m0;
        int p1 = cube2.springs[map2_springs[k]].m1;
        
        cube2.springs[map2_springs[k]].m0 = cube2.masses[p0].ID;
        cube2.springs[map2_springs[k]].m1 = cube2.masses[p1].ID;
        
        if (find(cube2.springIDs.begin(), cube2.springIDs.end(), cube1.springs[map1_springs[k]].ID) == cube2.springIDs.end()){
            cube2.springs[map2_springs[k]].ID = cube1.springs[map1_springs[k]].ID;
            cube2.springIDs.push_back(cube1.springs[map1_springs[k]].ID);
        }
        
        
        if (find(springs_left.begin(), springs_left.end(), map2_springs[k]) != springs_left.end()){
//            remove(springs_left.begin(), springs_left.end(), map2_springs[k]);
            int itr = find(springs_left.begin(), springs_left.end(), map2_springs[k])-springs_left.begin();
            springs_left.erase(springs_left.begin()+itr);
        }
    }
}

void initialize_cube(Cube &cube){
    vector<PointMass> masses;
    vector<Spring> springs;
    
    initialize_masses(masses);
    initialize_springs(springs);
    
    cube.masses = masses;
    cube.springs = springs;
    
    for (int i=0; i<6; i++){
        cube.free_faces.push_back(i);
    }
    
    float x_center = 0;
    float y_center = 0;
    float z_center = 0;
    for (int m=0; m<cube.masses.size(); m++){
        x_center += cube.masses[m].position[0];
        y_center += cube.masses[m].position[1];
        z_center += cube.masses[m].position[2];
    }
    
    x_center = x_center/cube.masses.size();
    y_center = y_center/cube.masses.size();
    z_center = z_center/cube.masses.size();
    
    cube.center = {x_center, y_center, z_center};
    
}

void initialize_masses(vector<PointMass> &masses){
    //Point Mass of bottom, front left vertex
    //----------------------
    PointMass mass0;
    mass0.mass = 1.0f;
    mass0.position = {-0.25f, -0.25f, 0.0f};
    mass0.velocity = {0.0f, 0.0f, 0.0f};
    mass0.acceleration = {0.0f, 0.0f, 0.0f};
    mass0.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of bottom, back left vertex
    //----------------------
    PointMass mass1;
    mass1.mass = 1.0f;
    mass1.position = {-0.25f, 0.25f, 0.0f};
    mass1.velocity = {0.0f, 0.0f, 0.0f};
    mass1.acceleration = {0.0f, 0.0f, 0.0f};
    mass1.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of bottom, back right vertex
    //----------------------
    PointMass mass2;
    mass2.mass = 1.0f;
    mass2.position = {0.25f, 0.25f, 0.0f};
    mass2.velocity = {0.0f, 0.0f, 0.0f};
    mass2.acceleration = {0.0f, 0.0f, 0.0f};
    mass2.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of bottom, front right vertex
    //----------------------
    PointMass mass3;
    mass3.mass = 1.0f;
    mass3.position = {0.25f, -0.25f, 0.0f};
    mass3.velocity = {0.0f, 0.0f, 0.0f};
    mass3.acceleration = {0.0f, 0.0f, 0.0f};
    mass3.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, front left vertex
    //----------------------
    PointMass mass4;
    mass4.mass = 1.0f;
    mass4.position = {-0.25f, -0.25f, 0.5f};
    mass4.velocity = {0.0f, 0.0f, 0.0f};
    mass4.acceleration = {0.0f, 0.0f, 0.0f};
    mass4.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, back left vertex
    //----------------------
    PointMass mass5;
    mass5.mass = 1.0f;
    mass5.position = {-0.25f, 0.25f, 0.5f};
    mass5.velocity = {0.0f, 0.0f, 0.0f};
    mass5.acceleration = {0.0f, 0.0f, 0.0f};
    mass5.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, back right vertex
    //----------------------
    PointMass mass6;
    mass6.mass = 1.0f;
    mass6.position = {0.25f, 0.25f, 0.5f};
    mass6.velocity = {0.0f, 0.0f, 0.0f};
    mass6.acceleration = {0.0f, 0.0f, 0.0f};
    mass6.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    //Point Mass of top, front right vertex
    //----------------------
    PointMass mass7;
    mass7.mass = 1.0f;
    mass7.position = {0.25f, -0.25f, 0.5f};
    mass7.velocity = {0.0f, 0.0f, 0.0f};
    mass7.acceleration = {0.0f, 0.0f, 0.0f};
    mass7.forces = {0.0f, 0.0f, 0.0f};
    //----------------------
    
    masses = {mass0, mass1, mass2, mass3, mass4, mass5, mass6, mass7};
    
}

void initialize_springs(vector<Spring> &springs){
    
    //Bottom Face of the Cube
    //-----------------------
    Spring spring0;
    spring0.L0 = 0.5f;
    spring0.L = 0.5f;
    spring0.k = spring_constant;
    spring0.m0 = 0;
    spring0.m1 = 1;
    spring0.original_L0 = 0.5f;
    
    Spring spring1;
    spring1.L0 = 0.5f;
    spring1.L = 0.5f;
    spring1.k = spring_constant;
    spring1.m0 = 1;
    spring1.m1 = 2;
    spring1.original_L0 = 0.5f;
    
    Spring spring2;
    spring2.L0 = 0.5f;
    spring2.L = 0.5f;
    spring2.k = spring_constant;
    spring2.m0 = 2;
    spring2.m1 = 3;
    spring2.original_L0 = 0.5f;
    
    Spring spring3;
    spring3.L0 = 0.5f;
    spring3.L = 0.5f;
    spring3.k = spring_constant;
    spring3.m0 = 3;
    spring3.m1 = 0;
    spring3.original_L0 = 0.5f;
    //----------------------
    
    //Cross Springs of Bottom Face
    //----------------------
    Spring spring4;
    spring4.L0 = 0.5f*sqrt(2.0f);
    spring4.L = 0.5f*sqrt(2.0f);
    spring4.k = spring_constant;
    spring4.m0 = 0;
    spring4.m1 = 2;
    spring4.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring5;
    spring5.L0 = 0.5f*sqrt(2.0f);
    spring5.L = 0.5f*sqrt(2.0f);
    spring5.k = spring_constant;
    spring5.m0 = 1;
    spring5.m1 = 3;
    spring5.original_L0 = 0.5f*sqrt(2.0f);
    //----------------------
    
    //Vertical Supports of Cube
    //----------------------
    Spring spring6;
    spring6.L0 = 0.5f;
    spring6.L = 0.5f;
    spring6.k = spring_constant;
    spring6.m0 = 0;
    spring6.m1 = 4;
    spring6.original_L0 = 0.5f;
    
    Spring spring7;
    spring7.L0 = 0.5f;
    spring7.L = 0.5f;
    spring7.k = spring_constant;
    spring7.m0 = 1;
    spring7.m1 = 5;
    spring7.original_L0 = 0.5f;
    
    Spring spring8;
    spring8.L0 = 0.5f;
    spring8.L = 0.5f;
    spring8.k = spring_constant;
    spring8.m0 = 2;
    spring8.m1 = 6;
    spring8.original_L0 = 0.5f;
    
    Spring spring9;
    spring9.L0 = 0.5f;
    spring9.L = 0.5f;
    spring9.k = spring_constant;
    spring9.m0 = 3;
    spring9.m1 = 7;
    spring9.original_L0 = 0.5f;
    //---------------------
    
    //Cross Springs of Front Face
    //---------------------
    Spring spring10;
    spring10.L0 = 0.5f*sqrt(2.0f);
    spring10.L = 0.5f*sqrt(2.0f);
    spring10.k = spring_constant;
    spring10.m0 = 0;
    spring10.m1 = 7;
    spring10.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring11;
    spring11.L0 = 0.5f*sqrt(2.0f);
    spring11.L = 0.5f*sqrt(2.0f);
    spring11.k = spring_constant;
    spring11.m0 = 3;
    spring11.m1 = 4;
    spring11.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Cross Springs of Left Face
    //---------------------
    Spring spring12;
    spring12.L0 = 0.5f*sqrt(2.0f);
    spring12.L = 0.5f*sqrt(2.0f);
    spring12.k = spring_constant;
    spring12.m0 = 0;
    spring12.m1 = 5;
    spring12.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring13;
    spring13.L0 = 0.5f*sqrt(2.0f);
    spring13.L = 0.5f*sqrt(2.0f);
    spring13.k = spring_constant;
    spring13.m0 = 1;
    spring13.m1 = 4;
    spring13.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Cross Springs of Back Face
    //---------------------
    Spring spring14;
    spring14.L0 = 0.5f*sqrt(2.0f);
    spring14.L = 0.5f*sqrt(2.0f);
    spring14.k = spring_constant;
    spring14.m0 = 1;
    spring14.m1 = 6;
    spring14.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring15;
    spring15.L0 = 0.5f*sqrt(2.0f);
    spring15.L = 0.5f*sqrt(2.0f);
    spring15.k = spring_constant;
    spring15.m0 = 2;
    spring15.m1 = 5;
    spring15.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Cross Springs of Right Face
    //---------------------
    Spring spring16;
    spring16.L0 = 0.5f*sqrt(2.0f);
    spring16.L = 0.5f*sqrt(2.0f);
    spring16.k = spring_constant;
    spring16.m0 = 2;
    spring16.m1 = 7;
    spring16.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring17;
    spring17.L0 = 0.5f*sqrt(2.0f);
    spring17.L = 0.5f*sqrt(2.0f);
    spring17.k = spring_constant;
    spring17.m0 = 3;
    spring17.m1 = 6;
    spring17.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Top Face of the Cube
    //---------------------
    Spring spring18;
    spring18.L0 = 0.5f;
    spring18.L = 0.5f;
    spring18.k = spring_constant;
    spring18.m0 = 4;
    spring18.m1 = 5;
    spring18.original_L0 = 0.5f;
    
    Spring spring19;
    spring19.L0 = 0.5f;
    spring19.L = 0.5f;
    spring19.k = spring_constant;
    spring19.m0 = 5;
    spring19.m1 = 6;
    spring19.original_L0 = 0.5f;
    
    Spring spring20;
    spring20.L0 = 0.5f;
    spring20.L = 0.5f;
    spring20.k = spring_constant;
    spring20.m0 = 6;
    spring20.m1 = 7;
    spring20.original_L0 = 0.5f;
    
    Spring spring21;
    spring21.L0 = 0.5f;
    spring21.L = 0.5f;
    spring21.k = spring_constant;
    spring21.m0 = 7;
    spring21.m1 = 4;
    spring21.original_L0 = 0.5f;
    //---------------------
    
    //Cross Springs of Top Face
    //---------------------
    Spring spring22;
    spring22.L0 = 0.5f*sqrt(2.0f);
    spring22.L = 0.5f*sqrt(2.0f);
    spring22.k = spring_constant;
    spring22.m0 = 4;
    spring22.m1 = 6;
    spring22.original_L0 = 0.5f*sqrt(2.0f);
    
    Spring spring23;
    spring23.L0 = 0.5f*sqrt(2.0f);
    spring23.L = 0.5f*sqrt(2.0f);
    spring23.k = spring_constant;
    spring23.m0 = 5;
    spring23.m1 = 7;
    spring23.original_L0 = 0.5f*sqrt(2.0f);
    //---------------------
    
    //Inner Cross Springs
    //---------------------
    Spring spring24;
    spring24.L0 = 0.5f*sqrt(3.0f);
    spring24.L = 0.5f*sqrt(3.0f);
    spring24.k = spring_constant;
    spring24.m0 = 0;
    spring24.m1 = 6;
    spring24.original_L0 = 0.5f*sqrt(3.0f);
    
    Spring spring25;
    spring25.L0 = 0.5f*sqrt(3.0f);
    spring25.L = 0.5f*sqrt(3.0f);
    spring25.k = spring_constant;
    spring25.m0 = 2;
    spring25.m1 = 4;
    spring25.original_L0 = 0.5f*sqrt(3.0f);
    
    Spring spring26;
    spring26.L0 = 0.5f*sqrt(3.0f);
    spring26.L = 0.5f*sqrt(3.0f);
    spring26.k = spring_constant;
    spring26.m0 = 1;
    spring26.m1 = 7;
    spring26.original_L0 = 0.5f*sqrt(3.0f);
    
    Spring spring27;
    spring27.L0 = 0.5f*sqrt(3.0f);
    spring27.L = 0.5f*sqrt(3.0f);
    spring27.k = spring_constant;
    spring27.m0 = 3;
    spring27.m1 = 5;
    spring27.original_L0 = 0.5f*sqrt(3.0f);
    //---------------------
    
    springs = {spring0, spring1, spring2, spring3, spring4, spring5, spring6, spring7, spring8, spring9, spring10, spring11, spring12, spring13, spring14, spring15, spring16, spring17, spring18, spring19, spring20, spring21, spring22, spring23, spring24, spring25, spring26, spring27};
}

void initialize_controller(Controller &control){
    for (int i=0; i<22; i++){
        Equation eqn;
        
        if (i==0){
            eqn.k = 1000;
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else if (i==1){
            eqn.k = 1000;
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else if (i==2){
            eqn.k = 1000;
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else if (i==3){
            eqn.k = 5000;
            eqn.a = 0.15;
            eqn.w = 2*M_PI;
            eqn.c = 0;
        }
        else if (i==4){
            eqn.k = 1000;
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }

        else if (i==5){
            eqn.k = 10000;
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else if (i==6){
            eqn.k = 5000;
            eqn.a = 0.1;
            eqn.w = M_PI;
            eqn.c = 0;
        }
        else if (i==7){
            eqn.k = 1000;
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else if (i==8){
            eqn.k = 1000;
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
        else if (i==9){
            eqn.k = 5000;
            eqn.a = 0.1;
            eqn.w = M_PI;
            eqn.c = 0;
        }
        else if (i==10){
            eqn.k = 1000;
            eqn.a = 0;
            eqn.w = 0;
            eqn.c = 0;
        }
//        else if (i==11){
//            eqn.k = 1000;
//            eqn.a = 0;
//            eqn.w = 0;
//            eqn.c = 0;
//        }
//        else if (i==12){
//            eqn.k = 1000;
//            eqn.a = 0;
//            eqn.w = 0;
//            eqn.c = 0;
//        }
//        else if (i==13){
//            eqn.k = 1000;
//            eqn.a = 0;
//            eqn.w = 0;
//            eqn.c = 0;
//        }
//        else if (i==14){
//            eqn.k = 10000;
//            eqn.a = 0;
//            eqn.w = 0;
//            eqn.c = 0;
//        }
//        else if (i==15){
//            eqn.k = 10000;
//            eqn.a = 0;
//            eqn.w = 0;
//            eqn.c = 0;
//        }
//        else if (i==16){
//            eqn.k = 5000;
//            eqn.a = 0.1;
//            eqn.w = 3;
//            eqn.c = M_PI;
//        }
//        else if (i==17){
//            eqn.k = 1000;
//            eqn.a = 0;
//            eqn.w = 0;
//            eqn.c = 0;
//        }
//        else if (i==18){
//            eqn.k = 1000;
//            eqn.a = 0;
//            eqn.w = 0;
//            eqn.c = 0;
//        }
//        else if (i==19){
//            eqn.k = 10000;
//            eqn.a = 0;
//            eqn.w = 0;
//            eqn.c = 0;
//        }
//        else if (i==20){
//            eqn.k = 5000;
//            eqn.a = 0.1;
//            eqn.w = 3;
//            eqn.c = 0;
//        }
//        else if (i==21){
//            eqn.k = 5000;
//            eqn.a = 0.2;
//            eqn.w = 3;
//            eqn.c = 0;
//        }
        control.motor.push_back(eqn);
    }
}

