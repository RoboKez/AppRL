//
//  EnvLinkSim.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 04/01/2023.
//

#include "EnvLinkSim.hpp"

/*-------------- LidarEnv--------------*/

LinkSimEnv::LinkSimEnv(std::string name, int m_seed, int step_limit)
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


void LinkSimEnv::GetOb()
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
    // Velocity
    float v_norm = 3.57;
    m_ob[6] = d->sensordata[8]/v_norm;
    m_ob[7] = d->sensordata[9]/-v_norm;
    m_ob[8] = d->sensordata[10]/v_norm;
    m_ob[9] = d->sensordata[11]/v_norm;
    // Orientation
    Euler tmp_e = QuaternionToEuler(d->sensordata[16], d->sensordata[17], d->sensordata[18], d->sensordata[19]);
    if(tmp_e.roll > 0){
        tmp_e.roll -= 180.0;
    } else {
        tmp_e.roll += 180.0;
    }
    m_ob[10] =tmp_e.roll/10.0;
    m_ob[11] = tmp_e.pitch/10.0;
    
//    // Position
//    float p_norm = 0.575959;
//    m_ob[2] = (-d->sensordata[0]/p_norm - 0.5)*2.0;
//    m_ob[3] = (d->sensordata[1]/p_norm  - 0.5)*2.0;
//    m_ob[4] = (-d->sensordata[2]/p_norm - 0.5)*2.0;
//    m_ob[5] = (-d->sensordata[3]/p_norm - 0.5)*2.0;
//    m_ob[6] = (d->sensordata[4]/p_norm - 0.5)*2.0;
//    m_ob[7] = (d->sensordata[5]/p_norm - 0.5)*2.0;
//    m_ob[8] = (d->sensordata[6]/p_norm - 0.5)*2.0;
//    m_ob[9] = (-d->sensordata[7]/p_norm - 0.5)*2.0;
//
//    // Velocity
//    float v_norm = 3.57;
//    m_ob[10] = d->sensordata[8]/v_norm;
//    m_ob[11] = d->sensordata[9]/-v_norm;
//    m_ob[12] = d->sensordata[10]/v_norm;
//    m_ob[13] = d->sensordata[11]/v_norm;
//    m_ob[14] = d->sensordata[12]/-v_norm;
//    m_ob[15] = d->sensordata[13]/-v_norm;
//    m_ob[16] = d->sensordata[14]/-v_norm;
//    m_ob[17] = d->sensordata[15]/v_norm;
//
//    // Orientation
//    Euler tmp_e = QuaternionToEuler(d->sensordata[16], d->sensordata[17], d->sensordata[18], d->sensordata[19]);
//    if(tmp_e.roll > 0){
//        tmp_e.roll -= 180.0;
//    } else {
//        tmp_e.roll += 180.0;
//    }
//    m_ob[18] =tmp_e.roll/10.0;
//    m_ob[19] = tmp_e.pitch/10.0;
    
    
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

void LinkSimEnv::GetOverlay(){
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


void LinkSimEnv::EpisodeReset(int startPos) {
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

void LinkSimEnv::TargetReset() {
    
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    float rand_posx = float(uni_dist(m_generator))/1000.0f -0.4;
    float rand_posy = float(uni_dist(m_generator))/1000.0f -0.4;
    d->qpos[9] = rand_posx;
    d->qpos[10] = rand_posy;
    mj_forward(m, d);
}

Eigen::VectorXf LinkSimEnv::NextOb() {
    return m_ob;
}

uint16_t LinkSimEnv::GetObSize() {
    return m_ob.size();
}

uint16_t LinkSimEnv::GetActSize() {
    return m_act_size;
}

LinkSimEnv::~LinkSimEnv()
{
    SHOW("LinkSimEnv destroyed");
}

//override env
void LinkSimEnv::MarkovStep(Eigen::VectorXf agent_act)
{
    d->ctrl[0] = agent_act[0];
    d->ctrl[1] = agent_act[1];
    d->ctrl[2] = agent_act[0];
    d->ctrl[3] = agent_act[1];

    d->ctrl[4] = -agent_act[2];
    d->ctrl[5] = -agent_act[3];
    d->ctrl[6] = -agent_act[2];
    d->ctrl[7] = -agent_act[3];

//    d->ctrl[0] = agent_act[0];
//    d->ctrl[1] = agent_act[1];
//    d->ctrl[2] = -agent_act[2];
//    d->ctrl[3] = -agent_act[3];
//
//    d->ctrl[4] = agent_act[2];
//    d->ctrl[5] = agent_act[3];
//    d->ctrl[6] = -agent_act[0];
//    d->ctrl[7] = -agent_act[1];
    
    sub_steps = 0;
    while (sub_steps < n_sub_steps and !m_quit){
        mj_step(m, d);
        sub_steps+=1;
        cam.fixedcamid =0;
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
        if(GLFWStuff::renderAll){Render();}
    }
    if(!GLFWStuff::renderAll){Render();}
    step +=1;
    GetOb(); // Get next observation, rewards and done
    
    if(m_done)Render();
    
}

