//
//  EnvLidar.cpp
//  RL_Bytes
//
//  Created by Kez Smithson Whitehead on 14/03/2022.
//

#include "EnvLidar.hpp"

/*-------------- LidarEnv--------------*/

LidarEnv::LidarEnv(std::string name, int m_seed, int step_limit)
    : MjEnv(name, step_limit)
{
    m_generator.seed(m_seed);
    cam.distance = 5;
    m_ob = Eigen::VectorXf(m_ob_size);  // ob size
    m_act = Eigen::VectorXf(m_act_size);  // act size
    
    pidX = PID(m->opt.timestep*n_sub_steps);
    pidY = PID(m->opt.timestep*n_sub_steps);
    
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
}


void LidarEnv::GetOb()
{
    m_reward = 0;
    // Observation -----------------------------------------------------
    float min_lidar = 10;
    float min_lidar_id = -1;
    for(int i=0; i<m->nsensordata; i++) {
        if (d->sensordata[i] < min_lidar){
            if (d->sensordata[i] > 0){  // as -1 indicates +inf
                min_lidar = d->sensordata[i];
                min_lidar_id = i;
            }
        }
    }
    min_lidar*=100; // scale to byte
    if(step==1 or min_lidar == 10)
        prev_min_lidar = min_lidar;

    if (min_lidar > 255) min_lidar = 255;
    if (min_lidar_id==-1) min_lidar_id = 255;

    m_ob[0] = min_lidar_id/100;
    m_ob[1] = min_lidar/255;
    
    m_reward = (prev_min_lidar - min_lidar)*0.001;

    // Reward ----------------------------------------------------------
//    if (target_moved or abs(prev_min_lidar - min_lidar)>10) {
//        m_reward = 0;
//        target_moved = false;
//    } else m_reward = (prev_min_lidar - min_lidar);
////    m_reward -= 2.0f; // agents life is constant pain...
//    prev_min_lidar = min_lidar;  // required for reward
//
//    if(m_reward < -127){
//        m_reward = -127;
//    } else if (m_reward > 127){
//        m_reward = 127;
//    }
    
    // Done ------------------------------------------------------------
    m_done = 0;
    if (min_lidar < 10) {
        m_reward +=10;
        m_done = true;  // done flag if 1 or target move flag if 2
    }
    if(step > StepLimit){
        m_done = true;
    }
}

void LidarEnv::GetOverlay(){
    label =  "Act: " + std::to_string((d->ctrl[0])) + "  " + std::to_string((d->ctrl[1])) +
    "\nOb: " + std::to_string((m_ob[0])) + "  " + std::to_string((m_ob[1])) +
    "\nRew: " + std::to_string((m_reward)) +
    "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
    "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
    "\nStep " +  std::to_string(step);
}


void LidarEnv::EpisodeReset(int startPos) {

    d->qpos[0] = 0;
    d->qpos[1] = 0;
    d->qpos[2] = 0.17;
    d->qpos[3] = 0;
    d->qpos[4] = 0;
    d->qpos[5] = 0;
    d->qvel[0] = 0;
    d->qvel[1] = 0;
    d->qvel[2] = 0;
    d->qvel[3] = 0;
    d->qvel[4] = 0;
    d->qvel[5] = 0;
    
    TargetReset();
    
    target_moved = true;
    m_done = false;
    step = -1;
    GetOb();
}

void LidarEnv::TargetReset() {
    
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    float rand_posx = float(uni_dist(m_generator))/1000.0f -0.4;
    float rand_posy = float(uni_dist(m_generator))/1000.0f -0.4;
    d->qpos[9] = rand_posx;
    d->qpos[10] = rand_posy;
    mj_forward(m, d);
}

Eigen::VectorXf LidarEnv::NextOb() {
    return m_ob;
}

uint16_t LidarEnv::GetObSize() {
    return m_ob.size();
}

uint16_t LidarEnv::GetActSize() {
    return m_act_size;
}

LidarEnv::~LidarEnv()
{
    SHOW("LidarEnv destroyed");
}
