//
//  EnvLinkNone.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 07/12/2022.
//

#include "EnvLinkLim.hpp"

/*-------------- LidarEnv--------------*/

LinkLimEnv::LinkLimEnv(std::string name, int m_seed, int step_limit)
    : MjEnv(name, step_limit)
{
    n_sub_steps = 4;
    m_generator.seed(m_seed);
    cam.distance = 5;
    cam.type = 2;
    m_ob = Eigen::VectorXf(m_ob_size);  // ob size
    m_act = Eigen::VectorXf(m_act_size);  // act size
    
    pidX = PID(m->opt.timestep*n_sub_steps);
    pidY = PID(m->opt.timestep*n_sub_steps);
    
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
}


void LinkLimEnv::GetOb()
{
    m_reward = 0;
    // Observation -----------------------------------------------------
    
    // Lidar
    float min_lidar_id = -1;
    int lidar_start = 20;
    float lidar_range = 2.5;
    float min_lidar = lidar_range;
    for(int i=lidar_start; i<m->nsensordata; i++) {
        if (d->sensordata[i] < min_lidar){
            if (d->sensordata[i] > 0){  // as -1 indicates +inf
                min_lidar = d->sensordata[i];
                min_lidar_id = i-lidar_start;
            }
        }
    }
    min_lidar_id = float(min_lidar_id)/100.0 - 1.8;
    m_ob[0] = min_lidar_id;
    m_ob[1] = min_lidar;
    
    // Position
    float p_norm = 0.575959;
    m_ob[2] = (-d->sensordata[0]/p_norm - 0.5)*2.0;
    m_ob[3] = (d->sensordata[1]/p_norm  - 0.5)*2.0;
    m_ob[4] = (-d->sensordata[2]/p_norm - 0.5)*2.0;
    m_ob[5] = (-d->sensordata[3]/p_norm - 0.5)*2.0;
    m_ob[6] = (d->sensordata[4]/p_norm - 0.5)*2.0;
    m_ob[7] = (d->sensordata[5]/p_norm - 0.5)*2.0;
    m_ob[8] = (d->sensordata[6]/p_norm - 0.5)*2.0;
    m_ob[9] = (-d->sensordata[7]/p_norm - 0.5)*2.0;
    
    // Velocity
    float v_norm = 3.57;
    m_ob[10] = d->sensordata[8]/v_norm;
    m_ob[11] = d->sensordata[9]/-v_norm;
    m_ob[12] = d->sensordata[10]/v_norm;
    m_ob[13] = d->sensordata[11]/v_norm;
    m_ob[14] = d->sensordata[12]/-v_norm;
    m_ob[15] = d->sensordata[13]/-v_norm;
    m_ob[16] = d->sensordata[14]/-v_norm;
    m_ob[17] = d->sensordata[15]/v_norm;
    
    // Orientation
    Euler tmp_e = QuaternionToEuler(d->sensordata[16], d->sensordata[17], d->sensordata[18], d->sensordata[19]);
    if(tmp_e.roll > 0){
        tmp_e.roll -= 180.0;
    } else {
        tmp_e.roll += 180.0;
    }
    m_ob[18] =tmp_e.roll/10.0;
    m_ob[19] = tmp_e.pitch/10.0;
    
    
    if(step<2){
        prev_min_lidar = m_ob[1];
        m_prev_ang = m_ob[0];
    }
    m_reward = (prev_min_lidar - m_ob[1]);
    if (abs(prev_min_lidar - m_ob[1])>0.2){
        m_reward = 0;
    }
    prev_min_lidar = m_ob[1];
    m_prev_ang = m_ob[0];
    
    // Done ------------------------------------------------------------
    m_done = 0;

    if(step > StepLimit){
        m_done = 2;
    }
    else if(m_ob[1]<0.1 && step>0){
        m_done = 1;
        m_reward+=1.0;
        SHOW("Target reached")
    }
//    else if(abs(m_ob[0])>0.45 && step>0){
//        m_done = 1;
//        SHOW("Turned to far")
//    }
}

void LinkLimEnv::GetOverlay(){
    label = "\nCtrl:\t";
    for(int i=0; i<GetActSize(); i++) {label += "  " + std::to_string((d->ctrl[i]));}
    
    label += "\nOb:\t";
    for(int i=0; i<GetObSize(); i++) {label += "  " + std::to_string((m_ob[i]));}
    
    std::string label_cont =
    "\nRew:\t" + std::to_string((m_reward)) +
    "\nFR\t" + std::to_string(int(GLFWStuff::framerate)) +
    "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
    "\nStep\t" +  std::to_string(step);
    label += label_cont;
}


void LinkLimEnv::EpisodeReset(int startPos) {
    simstart = 0;
    mj_resetData(m, d);
    if(startPos==1){WarmStart(50);}
    
    mj_forward(m, d);
    m_danger_zone_count = 0;
    step = -1;
    GetOb();
    m_done = false;
    step = -1;
    GetOb();
}

void LinkLimEnv::TargetReset() {
    
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    float rand_posx = float(uni_dist(m_generator))/1000.0f -0.4;
    float rand_posy = float(uni_dist(m_generator))/1000.0f -0.4;
    d->qpos[9] = rand_posx;
    d->qpos[10] = rand_posy;
    mj_forward(m, d);
}

Eigen::VectorXf LinkLimEnv::NextOb() {
    return m_ob;
}

uint16_t LinkLimEnv::GetObSize() {
    return m_ob.size();
}

uint16_t LinkLimEnv::GetActSize() {
    return m_act_size;
}

LinkLimEnv::~LinkLimEnv()
{
    SHOW("LidarEnv destroyed");
}


