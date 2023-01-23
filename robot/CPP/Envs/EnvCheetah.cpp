#include "EnvCheetah.hpp"
#include<cmath>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>
// frequency skips 5 i think with randomness
// add noise

CheetahEnv::CheetahEnv(std::string name, int m_seed, int step_limit)
    : MjEnv(name, step_limit)
{
    m_act_size = 6;
    m_ob_size = 17;
    m_ob = Eigen::VectorXf(m_ob_size);
    m_act = Eigen::VectorXf(m_act_size);
    m_generator.seed(m_seed);
    cam.distance = 5;
}


void CheetahEnv::GetOb()
{

    for (int i=0; i<8; i++){
        m_ob[i] = d->qpos[i+1];
    }
    for (int i=0; i<9; i++){
        m_ob[i+8] = d->qvel[i];
    }

    // Reward ----------------------------------------------------------
    float reward_ctrl = 0;
    for (int i=0; i==9; i++){reward_ctrl += pow(d->ctrl[i], 2);}
    reward_ctrl *= -0.1;
    float dt = 0.01*5; // as 5 mujoco steps = 1 markov step
    float reward_run = (d->qpos[0] - xposbefore)/dt;
    
    m_reward = reward_ctrl + reward_run;

    // Done ------------------------------------------------------------
    m_done = 0;
    if(step == StepLimit-1){
        m_done = true;
    }
    xposbefore = d->qpos[0];
}

void CheetahEnv::EpisodeReset(int startPos) {
    simstart = 0;
    mj_resetData(m, d);
    
    std::normal_distribution<double> noise_distribution_vel(0.0,0.01);
    std::uniform_int_distribution<double> noise_distribution_pos(-1000, 1000);
    
    for (int i=0; i==8; i++){
        d->qpos[i] += float(noise_distribution_pos(m_generator))/10000;
        d->qvel[i] += noise_distribution_vel(m_generator);
    } //check
    
    mj_forward(m, d);
    m_done = false;
    step = -1;
    xposbefore = d->qpos[0];
    GetOb();
}

Eigen::VectorXf CheetahEnv::NextOb() {
    return m_ob;
}

uint16_t CheetahEnv::GetObSize() {
    return m_ob.size();
}

uint16_t CheetahEnv::GetActSize() {
    return m_act_size;
}

CheetahEnv::~CheetahEnv()
{
    SHOW("CartPoleEnv destroyed");
}

void CheetahEnv::GetOverlay(){
    
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
//    SHOW2("steps", step)
}
