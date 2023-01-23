//
//  EnvPiper.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 11/09/2022.
//

#include "EnvPiper.hpp"


PiperEnv::PiperEnv(std::string name, int m_seed, int step_limit, int ob_mode)
    : MjEnv(name, step_limit), m_ob_mode(ob_mode)
{
    n_sub_steps =  5;
    m_act_size = 1;
    
    if(m_ob_mode==0){
        m_ob_name = "PD";
        m_ob_size = 2;
    }
    else if(m_ob_mode==1)
    {
        m_ob_name = "P";
        m_ob_size = 1;
    }
    else if(m_ob_mode==2)
    {
        m_ob_name = "PI";
        m_ob_size = 2;
    }
    else if(m_ob_mode==3)
    {
        m_ob_name = "PID";
        m_ob_size = 4;
        
    }
    else if(m_ob_mode==4)
    {
        m_ob_name = "Classic";
        m_ob_size = 4;
    } else if(m_ob_mode==5){
        m_ob_name = "All";
        m_ob_size = 4;
        m_act_size = 2;
    } else {
        SHOW("RL Error: Invalid ob mode")
    }
    SHOW2("Ob mode = ", m_ob_name)
    
    m_generator.seed(m_seed);
    cam.distance = 5;
    SHOW(m_ob_size);
    m_ob = Eigen::VectorXf(m_ob_size);  // ob size
    SHOW(m_ob);
    pidPole = PID(m->opt.timestep*n_sub_steps);
    pidCart = PID(m->opt.timestep*n_sub_steps);
}

void PiperEnv::EpisodeReset(int startPos)
{
//    GLFWStuff::sp2 = 2.0;
//    d->mocap_pos[0] = GLFWStuff::sp2;
//    GLFWStuff::sp2 = 2.3;
    d->mocap_pos[0] = GLFWStuff::sp2;
    
    m_prev_lidar_dist = 0;
    m_prev_lidar_ang = 0;
    
    simstart = 0;
    mj_resetData(m, d);
    
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    std::normal_distribution<> noise_dis(0.0, 0.01);
    if (startPos==1){  // uniform random
        d->qpos[m_ID] = float(uni_dist(m_generator))/(1000.0f*0.65f*2.0);
    } else if (startPos==2){  // fall position
        d->qpos[m_ID] = 0.65f/4.0;
    }
    
    
//    d->mocap_pos[1] = 0.5;
    
//    d->mocap_pos[0] = float(uni_dist(m_generator))/(500.0f)-1.0f;
//    d->mocap_pos[1] = float(uni_dist(m_generator))/(500.0f)-1.0f;
    SHOW2("\nreset angle ", d->qpos[m_ID]);
    SHOW2("sensor", d->sensordata[2]);
    
    mj_forward(m, d);
    m_danger_zone_count = 0;
    step = -1;
    pidPole.Reset();
    pidCart.Reset();
    GetOb();
    m_reward = 0;
}

void PiperEnv::GetOb()
{
    MinLidar();
    std::normal_distribution<> noise_dis(0.0, m_sensor_noise);
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    
    Euler tmp_e = QuaternionToEuler(d->sensordata[7], d->sensordata[8], d->sensordata[9], d->sensordata[10]);
    m_true = tmp_e.pitch/rad_ratio;
    tmp_e.pitch = 0;
    tmp_e.roll = 0;
//    tmp_e.yaw //bearing
    Quaternion tmp_q = EulerToQuaternion(tmp_e.roll, tmp_e.pitch, tmp_e.yaw);
    
    d->mocap_quat[4] = tmp_q.w;
    d->mocap_quat[5] = tmp_q.x;
    d->mocap_quat[6] = tmp_q.y;
    d->mocap_quat[7] = tmp_q.z; //14
    d->mocap_pos[3] = d->sensordata[11];
    d->mocap_pos[4] = d->sensordata[12];

    // Pole Pos -----------------------------------------
//    float h = (d->sensordata[16] - d->sensordata[13]);
//    float w = (d->sensordata[14] - d->sensordata[11]);
//    float ang = 0;
//    float norm_pitch = 1.0;
//    if(h==0){
//        ang = 0;
//    }else if(h<=0 && w>0){
//        ang=m_kill_angle*norm_pitch;
//
//    }else if(h<=0 && w<0){
//        ang=-m_kill_angle*norm_pitch;
//    }else{
//        SHOW2("w", w)
//        SHOW2("w", w)
//        ang = std::tan(w/h);
//    }
//    m_true = ang/norm_pitch;
    
    
    
    
    // Cart Pos -----------------------------------------
    float wheelPos = ((d->sensordata[0] + d->sensordata[1])/2.0);
//    float cartPos = wheelPos-m_true;
                              
    pidPole.Update(m_true, 0); //GLFWStuff::sp intead of 0
    float poleVel = (m_prev_pitch - m_true);
    Euler eul = QuaternionToEuler(d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
    m_yaw = eul.yaw/rad_ratio;
    m_ob[0] = pidPole.P;
    m_ob[1] = poleVel*5.0; //pidPole.D;
    m_ob[2] = m_lidar_ang;
    m_ob[3] = m_lidar_dist;
//    m_ob[4] = m_yaw;


//    m_reward = ((1.0/(1.0 + abs(pidPole.P)))-0.5f) * 0.01;
//    m_reward += ((1.0/(1.0 + abs(m_ob[2])))-0.5f) * 0.01;
    m_done = 0;
    if (abs(m_true) >= m_kill_angle){
        m_done = 1;
        SHOW("fell over")
        SHOW2("kill", m_kill_angle)
        SHOW2("m_true", m_true)
        
    } else if (step >= StepLimit-1){
        m_done = 2;
    }
    m_prev_pitch = m_true;
}


void PiperEnv::MinLidar()
{
    float min_lidar = 10;
    float min_lidar_id = -1;
    for(int i=17; i<377; i++) {
        if (d->sensordata[i] < min_lidar){
            if (d->sensordata[i] > 0){  // as -1 indicates +inf
                min_lidar = d->sensordata[i];
                min_lidar_id = i-17;
            }
        }
    }
    
//    m_reward = abs(m_prev_lidar_dist) - abs(m_lidar_dist);
    m_reward = ((1.0/(1.0 + abs(m_lidar_dist)))-0.5f) * 0.01;
    m_reward += ((1.0/(1.0 + abs(pidPole.P)))-0.5f) * 0.01;
//    m_reward += abs(m_prev_lidar_ang) - abs(m_lidar_ang);
//    m_reward +=  0.01/(1.0 + abs(m_yaw));
    if (step<2){m_reward=0;}
    
    SHOW2("rew", m_reward);
    SHOW2("step", step);
    m_prev_lidar_ang = m_lidar_ang;
    m_prev_lidar_dist = m_lidar_dist;
    
    m_lidar_ang = (min_lidar_id-180.0)/180.0;
    m_lidar_dist = min_lidar-0.5;
    SHOW2("bear",m_lidar_ang)
    SHOW2("distance",m_lidar_dist)
    
}



void PiperEnv::GetOverlay(){
    
    d->mocap_pos[0] = GLFWStuff::sp2;

    Euler tmp_e = QuaternionToEuler(d->sensordata[7], d->sensordata[8], d->sensordata[9], d->sensordata[10]);
    float tmp_pitch = tmp_e.pitch;
    tmp_e.pitch = 0;
    tmp_e.roll = 0;
//    tmp_e.yaw //bearing
    Quaternion tmp_q = EulerToQuaternion(tmp_e.roll, tmp_e.pitch, tmp_e.yaw);
    d->mocap_quat[4] = tmp_q.w;
    d->mocap_quat[5] = tmp_q.x;
    d->mocap_quat[6] = tmp_q.y;
    d->mocap_quat[7] = tmp_q.z; //14
    d->mocap_pos[3] = d->sensordata[11];
    d->mocap_pos[4] = d->sensordata[12];
    
    std::string det_string;
    if (m_deterministic) {
        det_string = "Deterministic";
    } else {
        det_string = "Stocastic";
    }
    
    std::string ob_show = "\nOb: ";
    std::string sp_show = "\nSet_Point" + std::to_string(GLFWStuff::sp);
    
    if(m_ob_mode==0){ob_show = ob_show + "P:"  + std::to_string(m_ob[0]) + "D:" + std::to_string(m_ob[1]);}
    else if(m_ob_mode==1){ob_show = ob_show + "P:"  + std::to_string(m_ob[0]);}
    else if(m_ob_mode==2){ob_show = ob_show + "P:"  + std::to_string(m_ob[0]) + "I:" + std::to_string(m_ob[1]);}
    else if(m_ob_mode==3){ob_show = ob_show + "P:"  + std::to_string(m_ob[0]) + "V:" + std::to_string(m_ob[1]) + "P:" + std::to_string(m_ob[2]) + "V:" + std::to_string(m_ob[3]);}
    else if(m_ob_mode==4){
        ob_show = ob_show + "PoleP:"  + std::to_string(m_ob[0]) + "PoleD:" + std::to_string(m_ob[1]) + "CartP:" + std::to_string(m_ob[2]) + "CartD:" + std::to_string(m_ob[3]) ;
        sp_show +=  ("\nSet_Point" + std::to_string(GLFWStuff::sp2));
        
    } else if(m_ob_mode==5){
        ob_show += "PoleP:"  + std::to_string(m_ob[0]) + "PoleD:" + std::to_string(m_ob[1]);
        ob_show += "LA:"  + std::to_string(m_ob[2]) + "LD:" + std::to_string(m_ob[3]);
//        ob_show += "Yaw:" + std::to_string(m_ob[4]);
        sp_show +=  ("\nSet_Point" + std::to_string(GLFWStuff::sp2));
        sp_show +=  ("\nlidar dis" + std::to_string(m_lidar_dist));
        
    }else {SHOW("RL Error: Invalid ob mode")}
    
    label =  "Act: " + std::to_string(d->ctrl[0]) +
    ob_show +
    "\nRew: " + std::to_string(m_reward) +
    "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
    "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
    "\nStep " +  std::to_string(step) +
    "\n" + det_string +
    sp_show +
    "\nCur_Point" + std::to_string(m_true);
}

Eigen::VectorXf PiperEnv::NextOb() {
    return m_ob;
}

uint16_t PiperEnv::GetObSize() {
    return m_ob_size;
}

uint16_t PiperEnv::GetActSize() {
    return m_act_size;
}

PiperEnv::~PiperEnv()
{
    SHOW("PiperEnv destroyed");
}
