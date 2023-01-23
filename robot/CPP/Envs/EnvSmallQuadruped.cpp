//
//  SmallQuadruped.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 05/12/2022.
//

#include "EnvSmallQuadruped.hpp"

/*-------------- LidarEnv--------------*/

SmallQuadrupedEnv::SmallQuadrupedEnv(std::string name, int m_seed, int step_limit)
    : MjEnv(name, step_limit)
{
    m_generator.seed(m_seed);
    n_sub_steps = 20;
    cam.distance = 5;
    m_ob = Eigen::VectorXf(m_ob_size);  // ob size
    m_act = Eigen::VectorXf(m_act_size);  // act size
    
    pidX = PID(m->opt.timestep*n_sub_steps);
    pidY = PID(m->opt.timestep*n_sub_steps);
    
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
}


void SmallQuadrupedEnv::GetOb()
{
    m_reward = 0;
    // Observation -----------------------------------------------------
    float min_lidar = 10;
    float min_lidar_id = -1;
    for(int i=20; i<m->nsensordata; i++) {
        if (d->sensordata[i] < min_lidar){
            if (d->sensordata[i] > 0){  // as -1 indicates +inf
                min_lidar = d->sensordata[i];
                min_lidar_id = i;
            }
        }
    }
    min_lidar*=100; // scale to byte
    if (min_lidar > 255) min_lidar = 255;
    if (min_lidar_id==-1) min_lidar_id = 255;
    
    m_ob[0] = min_lidar_id/100.0-1.1;
    if(m_ob[0]>1.8){m_ob[0] -= 3.7;}
    
    Euler tmp_e = QuaternionToEuler(d->sensordata[16], d->sensordata[17], d->sensordata[18], d->sensordata[19]);
    if(tmp_e.roll > 0){
        tmp_e.roll -= 180.0;
    } else {
        tmp_e.roll += 180.0;
    }
    
    m_ob[1] = min_lidar/125.5;
    m_ob[2] = ModulusMujoco(d->sensordata[0]);
    m_ob[3] = ModulusMujoco(d->sensordata[1], true);
    m_ob[4] = ModulusMujoco(d->sensordata[2]);
    m_ob[5] = ModulusMujoco(d->sensordata[3]);
    m_ob[6] = ModulusMujoco(d->sensordata[4], true);
    m_ob[7] = ModulusMujoco(d->sensordata[5], true);
    m_ob[8] = ModulusMujoco(d->sensordata[6], true);
    m_ob[9] = ModulusMujoco(d->sensordata[7]);
    
    m_ob[10] = d->sensordata[8];
    m_ob[11] = d->sensordata[9];
    m_ob[12] = d->sensordata[10];
    m_ob[13] = d->sensordata[11];
    m_ob[14] = d->sensordata[12];
    m_ob[15] = d->sensordata[13];
    m_ob[16] = d->sensordata[14];
    m_ob[17] = d->sensordata[15];
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
//    m_reward += (abs(m_prev_ang) - abs(m_ob[0]))*0.1;
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
    else if(abs(m_ob[0])>0.45 && step>0){
        m_done = 1;
        SHOW("Turned to far")
    }
}

void SmallQuadrupedEnv::GetOverlay(){
    label =  "Act: " + std::to_string((d->ctrl[0])) + "  " + std::to_string((d->ctrl[1])) +
    "\nOb: " + std::to_string((m_ob[0])) + "  " + std::to_string((m_ob[1])) + "  " + std::to_string((m_ob[10])) + "  " + std::to_string((m_ob[11])) + " " +
    std::to_string((m_ob[12])) + "  " + std::to_string((m_ob[5])) + "  " +
    std::to_string((m_ob[6])) + "  " + std::to_string((m_ob[7])) + "  " +
    std::to_string((m_ob[8])) + "  " + std::to_string((m_ob[9])) + "  " +
    "\nRew: " + std::to_string((m_reward)) +
    "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
    "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
    "\nStep " +  std::to_string(step);
}


void SmallQuadrupedEnv::EpisodeReset(int startPos) {
    simstart = 0;
    mj_resetData(m, d);
//    TargetReset();
//    WarmStart(10);
    
    mj_forward(m, d);
    m_danger_zone_count = 0;
    step = -1;
    GetOb();
    m_done = false;
    step = -1;
    GetOb();
}

void SmallQuadrupedEnv::TargetReset() {
    
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    float rand_posx = float(uni_dist(m_generator))/1000.0f;
    float rand_posy = float(uni_dist(m_generator))/1000.0f;
    d->mocap_pos[1]  = rand_posx;
    d->mocap_pos[0] = rand_posy;
    mj_forward(m, d);
}

Eigen::VectorXf SmallQuadrupedEnv::NextOb() {
    return m_ob;
}

uint16_t SmallQuadrupedEnv::GetObSize() {
    return m_ob.size();
}

uint16_t SmallQuadrupedEnv::GetActSize() {
    return m_act_size;
}

SmallQuadrupedEnv::~SmallQuadrupedEnv()
{
    SHOW("LidarEnv destroyed");
}
