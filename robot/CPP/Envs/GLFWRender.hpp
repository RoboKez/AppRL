//
//  GLFWRender.hpp
//  RL_Bytes
//
//  Created by Kez Smithson Whitehead on 15/03/2022.
//

#ifndef GLFWRender_hpp
#define GLFWRender_hpp

#include <stdio.h>
#include <string>
#include <iostream>

// OpenGL
#include "GLFWRender.hpp"


// OpenGL
//#include <GL/glew.h>
#include <GLFW/glfw3.h>

// Mujoco
#include "uitools.h"

#define SHOW(x) std::cout << x << std::endl;
#define SHOW2(x, y) std::cout << x << y << std::endl;


struct GLFWStuff
{
    GLFWStuff();
    static void KeyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void MouseButtonCallback(GLFWwindow* window, int button, int act, int mods);
    static void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
    static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    
    static uint8_t camView;
    static float framerate;
    static bool ScrollCam;
    static double ScrollCam_yoffset;
    static bool button_left;
    static bool button_middle;
    static bool button_right;
    static double lastx;
    static double lasty;
    static bool MoveCam;
    static double x_h;
    static double y_h;
    static mjtMouse mouse_action;
    static float sp;
    static float sp2;
    static bool renderAll;
    static bool help;
    static int windowCount;
    static bool driveMode;
//    static double lastSim;
};

#endif /* GLFWRender_hpp */
