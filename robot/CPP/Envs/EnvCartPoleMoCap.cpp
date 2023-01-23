//
//  EnvCartPoleMoCap.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 09/07/2022.
//

#include "EnvCartPoleMoCap.hpp"
#include<cmath>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>


CartPoleMoCap::CartPoleMoCap(std::string name, int m_seed, int step_limit, std::string env_mode)
    : MjEnv(name, step_limit)
{
    SHOW(env_mode)
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
    cam.distance = 5;
    m_env_mode = env_mode;
    if(m_env_mode == "cartpole")m_ob_size = 2;
    
    
    
    m_ob = Eigen::VectorXf(m_ob_size); 
    m_act = Eigen::VectorXf(m_act_size);
}

void CartPoleMoCap::GetOb()
{
    SHOW("Not implimented in MoCap")
}

void CartPoleMoCap::GetOverlay(){
    
    if(m_env_mode == "cartpole"){
        label =  "Act: " + std::to_string(m_act[0]) +
        "\nOb PenP:" + std::to_string(m_ob[0])+ " PenIntergral:" + std::to_string(m_ob[1])+
        "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
        "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
        "\nStep " +  std::to_string(step) +
        "\nSet_Point" + std::to_string(m_set_point) +
        "\nCur_Point" + std::to_string(m_mocap_pitch);
    } else {
        label =  "Act: " + std::to_string(m_act[0]) +
        "\nOb PenP:" + std::to_string(m_ob[0])+ " PenV:" + std::to_string(m_ob[1])+ " CartP:" +std::to_string(m_ob[3]) + + " CartV:" +std::to_string(m_ob[2]) +
        "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
        "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
        "\nStep " +  std::to_string(step) +
        "\nSet_Point" + std::to_string(m_set_point) +
        "\nCur_Point" + std::to_string(m_mocap_pitch);
    }
    
    
    
}

void CartPoleMoCap::MoCapStep(float pitch, float pos)
{
    float previous_pitch = m_mocap_pitch;
    float previous_position = m_mocap_position;
    
    m_mocap_pitch = pitch;
    m_mocap_position = pos;
    m_set_point = GLFWStuff::sp;
    
    float pitch_sub = (m_mocap_pitch - previous_pitch)/float(n_sub_steps);
    float pos_sub = (m_mocap_position - previous_position)/float(n_sub_steps);
    
    sub_steps = 0;
    while (sub_steps < n_sub_steps and !m_quit){
        
        Quaternion q = EulerToQuaternion(0, previous_pitch+sub_steps*pitch_sub, 0);
        d->mocap_quat[4] = q.x;
        d->mocap_quat[5] = q.y;
        d->mocap_quat[6] = q.z;
        d->mocap_quat[7] = q.w;
        
        d->mocap_pos[3] = previous_position + sub_steps * pos_sub;
        d->mocap_pos[6] = previous_position + sub_steps * pos_sub;
        
        mj_step(m, d);
        sub_steps+=1;
        cam.fixedcamid = 0;
        cam.trackbodyid = 3;
        cam.type = GLFWStuff::camView;
        if (GLFWStuff::ScrollCam){
            GLFWStuff::ScrollCam = false;
            mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*GLFWStuff::ScrollCam_yoffset, &scn, &cam);
        }
        if (GLFWStuff::MoveCam){
            GLFWStuff::MoveCam = false;
            mjv_moveCamera(m, GLFWStuff::mouse_action, GLFWStuff::x_h, GLFWStuff::y_h, &scn, &cam);
        }
        Render();
    }
    step +=1;
}


void CartPoleMoCap::EpisodeReset(int startPos) {
    m_ob = Eigen::VectorXf(m_ob_size);  //ob size
    simstart = 0;
    mj_resetData(m, d);
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
//    d->qpos[1] = float(uni_dist(m_generator))/10000.0f*2.0f;
    d->qpos[1] = -0.6;
    SHOW2("MOCAP reset angle", d->qpos[1])
    mj_forward(m, d);
    m_done = 0;
    step = -1;
    
    m_set_point = GLFWStuff::sp;
    m_true = d->qpos[1];
    float pitch_err = m_true - m_set_point;
    m_mocap_pitch = pitch_err;
    
    
    if(m_env_mode == "cartpole"){
        m_ob[0] = pitch_err;
        m_ob[1] = 0;
    } else {
        m_ob[0] = pitch_err;
        m_ob[1] = d->sensordata[1];
        m_ob[2] = d->sensordata[2];
        m_ob[3] = d->sensordata[3];
    }
    m_act[0] = 0;
    SHOW2("reset ob", m_ob)
}


Eigen::VectorXf CartPoleMoCap::NextOb() {
    return m_ob;
}

uint16_t CartPoleMoCap::GetObSize() {
    return m_ob_size;
}

uint16_t CartPoleMoCap::GetActSize() {
    return m_act_size;
}

CartPoleMoCap::~CartPoleMoCap()
{
    SHOW("CartPoleMoCap destroyed");
}


