//
//  EnvDrifter.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 15/07/2022.
//

#include "EnvDrifter.hpp"

DrifterEnv::DrifterEnv(std::string name, int m_seed, int step_limit)
    : MjEnv(name, step_limit)
{
    m_generator.seed(m_seed);
    cam.distance = 5;
    m_ob = Eigen::VectorXf(m_ob_size);  // ob size
    m_act = Eigen::VectorXf(m_act_size);  // ob size
    m_ID1 = 3;
    m_ID2 = 2;
    
    pidX = PID(m->opt.timestep*n_sub_steps);
    pidY = PID(m->opt.timestep*n_sub_steps);
}

void DrifterEnv::EpisodeReset(int startPos)
{
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    std::normal_distribution<> noise_dis(0.0, 0.01);
    
    simstart = 0;
    mj_resetData(m, d);
    if (startPos==1){  // uniform random
        d->qpos[m_ID1] = float(uni_dist(m_generator))/1000.0f*0.65f;
        d->qpos[m_ID2] = float(uni_dist(m_generator))/1000.0f*0.65f;
    } else if (startPos==2){  // fall position
        d->qpos[m_ID1] = 0.65f;
        d->qpos[m_ID2] = 0.65f;
    }
    mj_forward(m, d);
    pidX.Reset();
    pidY.Reset();
    step = -1;
    GetOb();
    
    SHOW2("\nReset angle x:", d->qpos[m_ID1])
    SHOW2("\nReset angle y:", d->qpos[m_ID2])
}


void DrifterEnv::GetOb()
{
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    std::normal_distribution<> noise_dis(0.0, m_sensor_noise);
    
    m_true = d->sensordata[0];
    pidX.Update(d->sensordata[0], GLFWStuff::sp);
    pidY.Update(d->sensordata[4], GLFWStuff::sp2);
    m_ob[0] = pidX.P;
    m_ob[1] = pidX.I;
    m_ob[2] = pidY.P;
    m_ob[3] = pidY.I;
    m_reward = (((m_kill_angle/(m_kill_angle + abs(pidX.P)))-0.5f) +
               ((m_kill_angle/(m_kill_angle + abs(pidY.P)))-0.5f)) * 0.01;

    // Done ------------------------------------------------------------
    m_done = 0;
    if ((abs(m_ob[0]) > m_kill_angle) or  (abs(m_ob[2]) > m_kill_angle)){
        m_done = 1;
    } else if (step >= StepLimit-1){
        m_done = 2;
    }
}

void DrifterEnv::GetOverlay(){
    
    std::string det_string;
    if (m_deterministic) {
        det_string = "Deterministic";
    } else {
        det_string = "Stocastic";
    }
    label =  "Act: " + std::to_string(d->ctrl[0]) + " " + std::to_string(d->ctrl[1]) +
    "\nOb: PX" + std::to_string(m_ob[0])+ " IX" + std::to_string(m_ob[1]) + " PY" + std::to_string(m_ob[2]) + " IY" + std::to_string(m_ob[3]) +
    "\nRew: " + std::to_string(m_reward) +
    "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
    "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
    "\nStep " +  std::to_string(step) +
    "\n" + det_string +
    "\nSet_Point" + std::to_string(GLFWStuff::sp) + " " + std::to_string(GLFWStuff::sp2) +
    "\nCur_Point" + std::to_string(d->sensordata[0]) + " " + std::to_string(d->sensordata[4]);
}


Eigen::VectorXf DrifterEnv::NextOb() {
    return m_ob;
}

uint16_t DrifterEnv::GetObSize() {
    return m_ob_size;
}

uint16_t DrifterEnv::GetActSize() {
    return m_act_size;
}

DrifterEnv::~DrifterEnv()
{
    SHOW("CartPoleEnv destroyed");
}

