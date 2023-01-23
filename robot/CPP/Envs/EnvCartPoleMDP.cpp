//
//  EnvCartPoleMDP.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 17/06/2022.
//

#include "EnvCartPoleMDP.hpp"

#include<cmath>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>


CartPoleMDPEnv::CartPoleMDPEnv(std::string name, int m_seed, int step_limit)
    : MjEnv(name, step_limit)
{
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
    cam.distance = 5;
    
    m_ob = Eigen::VectorXf(m_ob_size);
    m_act = Eigen::VectorXf(m_act_size);
}


void CartPoleMDPEnv::GetOb()
{
    
    d->mocap_pos[0] = GLFWStuff::sp;
    m_pitch_set_point = GLFWStuff::sp;
    m_true = d->sensordata[0];
    
    float pitch_pos = d->sensordata[0];
    float pitch_vel = d->sensordata[1];
//    float pitch_acc = m_ob[1] - d->sensordata[1];
    float wheel_vel = d->sensordata[2];
    float wheel_pos = d->sensordata[3]-m_pitch_set_point;
//    float wheel_acc = m_ob[2] - d->sensordata[2];
    
    m_ob[0] = pitch_pos-m_pitch_set_point;
    m_ob[1] = pitch_vel;
    m_ob[2] = wheel_vel;
    m_ob[3] = wheel_pos;
//    m_ob[4] = wheel_acc;
    
    if(step==-1){m_prev_pitch = pitch_pos;}
//    m_reward = abs(m_prev_pitch)-abs(pitch_pos);
//    m_reward =m_kill_angle-abs(pitch_pos);
    m_reward =(m_kill_angle-abs(pitch_pos))*0.01;
//    m_reward =(0.1);//-abs(d->sensordata[3]-m_pitch_set_point))*0.0001;
    
    m_prev_pitch = pitch_pos;
    
    // Done ------------------------------------------------------------
    m_done = 0;
    if (abs(d->sensordata[0]) > m_kill_angle) {
        m_done = 1;
        SHOW("fell ovwe")
//        m_reward = ;
    } else if (step >= StepLimit-1){
        m_done = 2;
//        m_reward = -abs(wheel_vel);
    } else {
        m_done = 0;
    }

    
}

void CartPoleMDPEnv::GetOverlay(){
    
    std::string det_string;
    if (m_deterministic) {
        det_string = "Deterministic";
    } else {
        det_string = "Stocastic";
    }
    
    label =  "Act: " + std::to_string(d->ctrl[0]) +
    "\nOb: PenP:" + std::to_string(m_ob[0])+ " PenV.:" + std::to_string(m_ob[1])+ " CartP:" +std::to_string(m_ob[3]) + + " CartV:" +std::to_string(m_ob[2]) +
    "\nRew: " + std::to_string(m_reward) +
    "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
    "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
    "\nStep " +  std::to_string(step) +
    "\n" + det_string +
    "\nSet_Point" + std::to_string(m_pitch_set_point);
}


void CartPoleMDPEnv::EpisodeReset(int startPos) {
    GLFWStuff::sp = 0.0;
    
    m_ob = Eigen::VectorXf(m_ob_size);  //ob size
    simstart = 0;
    mj_resetData(m, d);
    
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
//    d->qpos[1] = float(uni_dist(m_generator))/10000.0f*2.0f;
    d->qpos[1] = m_start_pos;
    SHOW2("reset angle", d->qpos[1])

    mj_forward(m, d);
    m_done = 0;
    m_target_moved = true;
    step = -1;
    GetOb();
    
//    SHOW2("mob reset", m_ob)
}

void CartPoleMDPEnv::TargetReset(bool randomise) {
    float rand_ball_angle;
    if (randomise){
        SHOW("target reset")
        std::uniform_int_distribution<> uni_dist(-1000, 1000);
        rand_ball_angle = float(uni_dist(m_generator))/10000.0f*2.0f + 0.1;
    } else {
        rand_ball_angle = 0.1;
    }
    d->qpos[2] =  rand_ball_angle;
    mj_forward(m, d);
    m_target_moved = true;
}


Eigen::VectorXf CartPoleMDPEnv::NextOb() {
    return m_ob;
}

uint16_t CartPoleMDPEnv::GetObSize() {
    return m_ob_size;
}

uint16_t CartPoleMDPEnv::GetActSize() {
    return m_act_size;
}

CartPoleMDPEnv::~CartPoleMDPEnv()
{
    SHOW("CartPoleEnv destroyed");
}

