//
//  EnvDragger.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 07/04/2022.
//

#include "EnvDragger.hpp"

DraggerEnv::DraggerEnv(std::string name, int m_seed, int step_limit)
    : MjEnv(name, step_limit)
{
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
}


void DraggerEnv::GetOb()
{
    // Observation
    m_ob[0] = d->sensordata[0];
    m_ob[1] = d->sensordata[1];
    m_ob[2] = d->sensordata[2];
    m_ob[3] = d->sensordata[3];
    
    // Reward
    if (step == -1){
        m_position = d->sensordata[2];
        m_reward = 0;
    } else {
        m_reward =  m_position - d->sensordata[2];
        m_position = d->sensordata[2];
    }
    // Done
    if(step == StepLimit-1){
        m_done = true;
    }
}

void DraggerEnv::GetOverlay(){
    
    std::string det_string;
    if (m_deterministic) {
        det_string = "Deterministic";
    } else {
        det_string = "Stocastic";
    }
    
    label =  "Act: " + std::to_string(d->ctrl[0]) +
    "\nOb: " + std::to_string(m_ob[0]) +
    "\nRew: " + std::to_string(m_reward) +
    "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
    "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
    "\nStep " +  std::to_string(step) +
    "\n" + det_string;
}


void DraggerEnv::EpisodeReset(int startPos) {
    simstart = 0;
    mj_resetData(m, d);
    mj_forward(m, d);
    m_done = false;
    step = -1;
    GetOb();
}

Eigen::VectorXf DraggerEnv::NextOb() {
    return m_ob;
}

uint16_t DraggerEnv::GetObSize() {
    return m_ob.size();
}

uint16_t DraggerEnv::GetActSize() {
    return m_act_size;
}

DraggerEnv::~DraggerEnv()
{
    SHOW("DraggerEnv destroyed");
}
