//
//  GLFWRender.cpp
//  RL_Bytes
//
//  Created by Kez Smithson Whitehead on 15/03/2022.
//

#include "GLFWRender.hpp"

uint8_t GLFWStuff::camView = 0;
float GLFWStuff::framerate = 120;
bool GLFWStuff::ScrollCam;
double GLFWStuff::ScrollCam_yoffset;
bool GLFWStuff::button_left;
bool GLFWStuff::button_middle;
bool GLFWStuff::button_right;
double GLFWStuff::lastx;
double GLFWStuff::lasty;
bool GLFWStuff::MoveCam;
double GLFWStuff::x_h;
double GLFWStuff::y_h;
float GLFWStuff::sp = 0;
float GLFWStuff::sp2 = 0;
bool GLFWStuff::renderAll = true;
bool GLFWStuff::help = false;
mjtMouse GLFWStuff::mouse_action;
int GLFWStuff::windowCount= 0;
bool GLFWStuff::driveMode = false;
//double GLFWStuff::lastSim= 0;


/*-------------- GLFWStuff--------------*/

GLFWStuff::GLFWStuff(){
    SHOW("GLFW stuff init")
}



/* Keyboard callback (Changes desired framerate) */
void GLFWStuff::KeyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act==GLFW_PRESS && key==GLFW_KEY_F) {
        if (framerate > 2) framerate/=2;
    }
    if (act==GLFW_PRESS && key==GLFW_KEY_S) {
        if (framerate < 120) framerate*=2;
    }
    
    if (act==GLFW_PRESS && key==GLFW_KEY_0) {
        camView = 0;
    } else if (act==GLFW_PRESS && key==GLFW_KEY_1) {
        camView = 1;
    }else if (act==GLFW_PRESS && key==GLFW_KEY_2) {
        camView = 2;
    }
    

    if (driveMode){
        if (act==GLFW_PRESS && key==GLFW_KEY_DOWN) {
            sp = 0.4;
        } else if (act==GLFW_PRESS && key==GLFW_KEY_UP) {
            sp = -0.4;
        }
        
        if ( (act==GLFW_RELEASE && key==GLFW_KEY_DOWN) or (act==GLFW_RELEASE && key==GLFW_KEY_UP)){
            sp = 0.0;
        }
        
        if (act==GLFW_PRESS && key==GLFW_KEY_LEFT) {
            sp2 = 0.8;
        } else if (act==GLFW_PRESS && key==GLFW_KEY_RIGHT) {
            sp2 = -0.8;
        }
        
        if ( (act==GLFW_RELEASE && key==GLFW_KEY_LEFT) or (act==GLFW_RELEASE && key==GLFW_KEY_RIGHT)){
            sp2 = 0.0;
        }
    } else {
        if (act==GLFW_PRESS && key==GLFW_KEY_LEFT) {
            sp-=0.01;
            SHOW("left")
        } else if (act==GLFW_PRESS && key==GLFW_KEY_RIGHT) {
            sp+=0.01;
        }
    
        if (act==GLFW_PRESS && key==GLFW_KEY_DOWN) {
            sp2-=0.01;
        } else if (act==GLFW_PRESS && key==GLFW_KEY_UP) {
            sp2+=0.01;
        }
    
        if (act==GLFW_PRESS && key==GLFW_KEY_W) {
            sp = -2.0;
        } else if (act==GLFW_PRESS && key==GLFW_KEY_R){
            sp = 2.0;
        } else if (act==GLFW_PRESS && key==GLFW_KEY_E){
            sp = 0.0;
        }
    
        if (act==GLFW_PRESS && key==GLFW_KEY_T) {
            sp2 = -1.0;
        } else if (act==GLFW_PRESS && key==GLFW_KEY_U){
            sp2 = 1.0;
        } else if (act==GLFW_PRESS && key==GLFW_KEY_Q){
            sp2 = 0.0;
        }
        
        if (act==GLFW_PRESS && key==GLFW_KEY_Z) {
            renderAll = true;
            SHOW2("renderAll:", renderAll)
        } else if (act==GLFW_PRESS && key==GLFW_KEY_X){
            renderAll = false;
            SHOW2("renderAll:", renderAll)
        }
        
        if (act==GLFW_PRESS && key==GLFW_KEY_H) {
            if (help){
                help = false;
            } else {
                help = true;
            }
        }
    }
    
        
    
    
}

/* Mouse button callback*/
void GLFWStuff::MouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


/* Mouse move callback */
void GLFWStuff::MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine mouse_action based on mouse button
  if (button_right) {
      mouse_action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
      mouse_action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
      mouse_action = mjMOUSE_ZOOM;
  }

  // move camera
    x_h = dx/height;
    y_h = dy/height;
    MoveCam = true;
}


/* scroll callback */
void GLFWStuff::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
    ScrollCam = true;
    ScrollCam_yoffset = yoffset;
    
}
