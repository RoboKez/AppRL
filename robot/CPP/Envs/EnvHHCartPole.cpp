//
//  EnvHHCartPole.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 28/06/2022.
//

#include "EnvHHCartPole.hpp"

CartPoleEnvHH::CartPoleEnvHH(std::string name,
                             const char * resourcePath,
                             const char * jsonFilePri,
                             const char * jsonFileSec,
                             int iterPri,
                             int iterSec,
                             int m_seed,
                             int step_limit)
    : MjEnv(name, step_limit)

{
    m_ob = Eigen::VectorXf(m_ob_size);  //ob size
    m_act = Eigen::VectorXf(m_act_size);  //act size
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
    cam.distance = 5;
    m_ob = Eigen::VectorXf(m_ob_size);  // ob size
    LoadSubPolicy(resourcePath, jsonFilePri, jsonFileSec);
}


void CartPoleEnvHH::EpisodeReset(int startPos) {
    m_ob = Eigen::VectorXf(m_ob_size);  //ob size
    m_act = Eigen::VectorXf(m_act_size);  //act size
//    std::normal_distribution<double> distribution(0.0, 2.0);
//    GLFWStuff::sp = distribution(m_generator);
    simstart = 0;
    mj_resetData(m, d);
//    d->qpos[2] =  0.;
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
//    d->qpos[0] = float(uni_dist(m_generator))/10000.0f*2.0f;
//    d->qpos[1] = float(uni_dist(m_generator))/10000.0f*2.0f;
//    d->qvel[0] = float(uni_dist(m_generator))/10000.0f*10.0f;
//    d->qvel[1] = float(uni_dist(m_generator))/10000.0f*10.0f;
    GLFWStuff::sp = float(uni_dist(m_generator))/1000.0f*2.0f;
    
    m_P_pri = 0;
    m_I_pri = 0;
    m_D_pri = 0;
    
    m_prev_err_pri = 0;
    m_prevI_pri = 0;
    m_prevD_pri = 0;
    
    m_P_sec = 0;
    m_I_sec = 0;
    m_D_sec = 0;
    
    m_prev_err_sec = 0;
    m_prevI_sec = 0;
    m_prevD_sec = 0;
    
    m_P_ter = 0;
    m_I_ter = 0;
    m_D_ter = 0;
    
    m_prev_err_ter = 0;
    m_prevI_ter = 0;
    m_prevD_ter = 0;
    
    mj_forward(m, d);
    m_done = false;
    m_target_moved = true;
    step = -1;
    GetOb();
}


void CartPoleEnvHH::GetOb()
{
    float t = 1.0f/8.0f;
    m_set_point = GLFWStuff::sp/10.0f;  // cart postion
    d->mocap_pos[0] = GLFWStuff::sp;
    m_true = d->sensordata[3]/10.0f;
    float err = (m_true - m_set_point); // cart postion error
    SHOW2("err", err)
    m_P_ter = err;
    m_D_ter = (err - m_prev_err_ter) / t;
    m_I_ter = (err + m_prev_err_ter) * t * 0.5 + m_prevI_ter;
    
//    m_reward = 2.0f/(2.0f+abs(m_P_ter))
    m_reward = ((2.0/(2.0 + abs(m_P_ter)))-0.5f)*2.0f -0.9;
//    m_reward =  clip((0.001/abs(m_P_ter)), 0.0, 1.0);
//    m_reward = (abs(m_prev_err_ter)  - abs(m_P_ter))*0.1;
    
    m_prev_err_ter = err;
    m_prevI_ter = m_I_ter;
    m_prevD_ter = m_D_ter;
    
//    if(step%10==0 && step>1){
//        std::normal_distribution<double> distribution(0.0, 2.0);
//        GLFWStuff::sp = distribution(m_generator);
//    }
    m_ob[0] = m_P_ter;
    m_ob[1] = m_I_ter/10.0f;
//    m_ob[2] = m_D_ter;

    // Done ------------------------------------------------------------
    m_done = 0;
    if (abs(d->sensordata[0]) > m_kill_angle) {
        m_done = 1;
    } else if (step >= StepLimit-1){
        m_done = true;
    }
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
//    if(step%50==0)d->qpos[2] = float(uni_dist(m_generator))/10000.0f*0.5f + 0.1;
    
//    } else if (abs(m_P_ter) >= 4.0){
//        m_done = 1;
//    } else {
//        m_done = false;
//    }
}

void CartPoleEnvHH::MarkovStep(Eigen::VectorXf agent_act)
{
//    float tsec = 1.0f/8.0f;
//    float tpri = 1.0f/40.0f;
//    agent_act[0]/=10.0f;
    
    float safe_speed = 2.0;
    if(agent_act[0] > safe_speed){
        agent_act[0] = safe_speed;
    }else if(agent_act[0] < -safe_speed){
        agent_act[0] = -safe_speed;
    }
    m_act = agent_act;
    float target_velocity = agent_act[0];
    
    
    // Velocity contoller
    for(std::uint16_t i=0; i<m_rs; ++i){
        float vel_err = d->sensordata[2] - target_velocity;
        m_P_sec = vel_err;
        m_I_sec = (vel_err + m_prev_err_sec) * m_ts * 0.5 + m_prevI_sec;
        m_D_sec = (vel_err - m_prev_err_sec) / m_ts;
        m_prev_err_sec = vel_err;
        m_prevI_sec = m_I_sec;

        Eigen::VectorXf agent_ob_sec = Eigen::VectorXf(secNet->m_in_size);
        agent_ob_sec[0] = m_P_sec;
        agent_ob_sec[1] = m_I_sec/5.0f;
        
        secNet->m_in = agent_ob_sec;
        secNet->ForwardPropagate();
        float target_pitch = secNet->m_out[0]/=5.0f;
        target_pitch = std::max(-m_kill_angle, std::min(target_pitch, m_kill_angle));
       
        
        // Pitch Controller
        for(std::uint16_t i=0; i<m_rp; ++i){
            agent_act[0]/=10.0f;
            float pit_err = d->sensordata[0] - target_pitch;
            m_P_pri = pit_err;
            m_I_pri = (pit_err + m_prev_err_pri) * m_tp * 0.5 + m_prevI_pri;
            m_D_pri = (pit_err - m_prev_err_pri) / m_tp;
            m_prev_err_pri = pit_err;
            m_prevI_pri = m_I_pri;

            Eigen::VectorXf agent_ob_pri = Eigen::VectorXf(priNet->m_in_size);
            agent_ob_pri[0] = m_P_pri;
            agent_ob_pri[1] = m_I_pri;
            
            priNet->m_in = agent_ob_pri;
            priNet->ForwardPropagate();
            d->ctrl[0] = priNet->m_out[0];  // Target torque
        
        
            // Torque Env
            int sub_steps = 0;
            while (sub_steps < n_sub_steps and !m_quit){
                mj_step(m, d);
                Render();
                m_set_point = GLFWStuff::sp;
                if (GLFWStuff::ScrollCam){
                    GLFWStuff::ScrollCam = false;
                    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*GLFWStuff::ScrollCam_yoffset, &scn, &cam);
                }
                if (GLFWStuff::MoveCam){
                    GLFWStuff::MoveCam = false;
                    mjv_moveCamera(m, GLFWStuff::mouse_action, GLFWStuff::x_h, GLFWStuff::y_h, &scn, &cam);
                }
                sub_steps+=1;
                cam.trackbodyid = 3;
                cam.type = GLFWStuff::camView;
            }
        }
    }
    step +=1;
    GetOb(); // Get next observation, rewards and done
}


void CartPoleEnvHH::GetOverlay(){
    
    std::string det_string;
    if (m_deterministic) {
        det_string = "Deterministic";
    } else {
        det_string = "Stocastic";
    }
    
    label =  "Torque SP: " + std::to_string(d->ctrl[0]) + "Posiyion SP: " + std::to_string(m_act[0]) +
    "\nOb:P" + std::to_string(m_ob[0])+  " I" + std::to_string(m_ob[1]) + " D"  +
    "\nRew: " + std::to_string(m_reward) +
    "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
    "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps))) + "Hz  " + std::to_string(int(d->time)) + "s"
    "\nStep " +  std::to_string(step) +
    "\n" + det_string +
    "\nSet_Point" + std::to_string(m_set_point) +
    "\nCur_Point" + std::to_string(d->sensordata[0]);
}


void CartPoleEnvHH::LoadSubPolicy(const char * resourcePath,
                                  const char * jsonFilePri,
                                  const char * jsonFileSec,
                                  int iterPri,
                                  int iterSec){
    chdir(resourcePath);
    SHOW2("jsonFilePri", jsonFilePri)
    SHOW2("jsonFileSec", jsonFileSec)
    
    char iFilePri[100] = "policy_";
    strcat(iFilePri, jsonFilePri);
    std::ifstream ipri(iFilePri);
    nlohmann::json jp;
    ipri >> jp;
    if(iterPri==-1){iterPri = int(jp.size())-1;}
    
    priNet = new NeuralNetwork(jp[iterPri]["W1"][0].size(),
                               jp[iterPri]["W3"].size(),
                               jp[iterPri]["W1"].size(),
                               jp[iterPri]["W2"].size());
    priNet->LoadNetwork(resourcePath, iFilePri, -1);
    
    
    char iFileSec[100] = "policy_";
    strcat(iFileSec, jsonFileSec);
    std::ifstream isec(iFileSec);
    nlohmann::json js;
    isec >> js;
    if(iterSec==-1){iterSec = int(js.size())-1;}
    
    secNet = new NeuralNetwork(js[iterSec]["W1"][0].size(),
                               js[iterSec]["W3"].size(),
                               js[iterSec]["W1"].size(),
                               js[iterSec]["W2"].size());
    secNet->LoadNetwork(resourcePath, iFileSec, -1);
    
    
//    SHOW2("priNet", priNet->m_A1.size())
//    SHOW2("secNet", secNet->m_A1.size())
//
    SHOW2("\nMy primary json file:\t",iFilePri)
    SHOW2("pri iter", iterPri)
    SHOW2("pri ob_size:\t", jp[iterPri]["W1"][0].size())
    SHOW2("pri h1_size:\t", jp[iterPri]["W1"].size())
    SHOW2("pri h2_size:\t", jp[iterPri]["W2"].size())
    SHOW2("pri out_size:\t", jp[iterPri]["W3"].size())
    
    SHOW2("\nMy secondary json file:\t",iFileSec)
    SHOW2("sec iter", iterSec)
    SHOW2("sec ob_size:\t", js[iterSec]["W1"][0].size())
    SHOW2("sec h1_size:\t", js[iterSec]["W1"].size())
    SHOW2("sec h2_size:\t", js[iterSec]["W2"].size())
    SHOW2("sec out_size:\t", js[iterSec]["W3"].size())
}


Eigen::VectorXf CartPoleEnvHH::NextOb() {
    return m_ob;
}

uint16_t CartPoleEnvHH::GetObSize() {
    return m_ob.size();
}

uint16_t CartPoleEnvHH::GetActSize() {
    return m_act_size;
}

CartPoleEnvHH::~CartPoleEnvHH()
{
    SHOW("CartPoleEnvH destroyed");
}
