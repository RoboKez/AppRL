//
//  EnvHCartPole.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 09/06/2022.
//

#include "EnvHCartPole.hpp"

CartPoleEnvH::CartPoleEnvH(std::string name, const char * resourcePath, const char * jsonFile, int sub_iter, int m_seed, int step_limit)
    : MjEnv(name, step_limit)

{
    m_rp = 5;
    m_ob = Eigen::VectorXf(m_ob_size);  //ob size
    m_act = Eigen::VectorXf(m_act_size);  //act size
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
    cam.distance = 5;
    SHOW2("here", sub_iter)
    m_ob = Eigen::VectorXf(m_ob_size);  // ob size
    LoadSubPolicy(resourcePath, jsonFile);
    
    pidPri = PID(m->opt.timestep*n_sub_steps);
    pidSec = PID(m->opt.timestep*n_sub_steps*m_rp);
    
}


void CartPoleEnvH::EpisodeReset(int startPos) {
    
       GLFWStuff::sp2 = 0.0;
        simstart = 0;
        mj_resetData(m, d);
    //    d->mocap_pos[0] = GLFWStuff::sp2*2.0;
        std::uniform_int_distribution<> uni_dist(-1000, 1000);
        std::normal_distribution<> noise_dis(0.0, 0.01);
    
        std::uniform_int_distribution<> direct(-1, 1);
    float side = uni_dist(m_generator);

        
        if (startPos==0){  // fixed with little noise
            d->qpos[m_ID] = 0.0 + float(uni_dist(m_generator))/1000.0f*0.3f;
        } else if (startPos==1){  // uniform random
            d->qpos[m_ID] = 0.6f;
        } else if (startPos==2){  // uniform random
            if(side>0){
                d->qpos[m_ID] = -0.6f + float(uni_dist(m_generator))/1000.0f*0.03f;;
            } else {
                d->qpos[m_ID] = 0.6f + float(uni_dist(m_generator))/1000.0f*0.03f;;
            }
            
        }
        SHOW2("\nreset angle ", d->qpos[m_ID])
        
        mj_forward(m, d);
        d->mocap_pos[0] = GLFWStuff::sp2*2.0;
        m_danger_zone_count = 0;
        step = -1;
        pidPri.Reset();
        pidSec.Reset();
        GetOb();
}


void CartPoleEnvH::GetOb()
{
    std::normal_distribution<> noise_dis(0.0, m_sensor_noise);
    float pos = (d->sensordata[2] + noise_dis(m_generator));  // cart position
    pidSec.Update(pos, GLFWStuff::sp2);

    m_ob[0] = pidSec.P;
    m_ob[1] = pidSec.I;
    

    m_reward = ((1.0/(1.0 + abs(pidSec.P)))) * 0.01;
    
    m_done = 0;
    if (abs(d->sensordata[0]) > m_kill_angle){
        m_done = 1;
        SHOW("fell over")
    } else if (step >= StepLimit-1){
        m_done = 2;
    }
//    else if (abs(m_ob[0]) > 0.99){
//        SHOW("out of bounds")
//        m_done = 1;
//    }
}


void CartPoleEnvH::MarkovStep(Eigen::VectorXf agent_act)
{
    agent_act[0]/=10.0f;
    m_act = agent_act;
    float target_pitch = agent_act[0];
    
    for(std::uint16_t i=0; i<int(m_rp); ++i){
        m_true = -d->sensordata[0]*rad_ratio/30.0f; //must be same normaliser as trained with add noise
        
        pidPri.Update(m_true, target_pitch);
        Eigen::VectorXf sub_agent_ob = Eigen::VectorXf(priNet->m_in_size);
        sub_agent_ob[0] = pidPri.P;
        sub_agent_ob[1] = pidPri.I;
        
        priNet->m_in = sub_agent_ob;
        priNet->ForwardPropagate();
        d->ctrl[0] = priNet->m_out[0];
        
        int sub_steps = 0;
        while (sub_steps < n_sub_steps and !m_quit){
            mj_step(m, d);
            Render();
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
        
//        if(m_total_subs < m_log_size){
//            m_log.col(m_total_subs)[0] = m_total_subs;
//            m_log.col(m_total_subs)[1] = GLFWStuff::sp2;
//            m_log.col(m_total_subs)[2] = m_prevsp-m_ob[0];
//            m_log.col(m_total_subs)[3] = m_act[0];
//            m_log.col(m_total_subs)[4] = d->sensordata[0];
//            m_log.col(m_total_subs)[5] = priNet->m_out[0];
//            m_log.col(m_total_subs)[6] = step;
//            m_total_subs += 1;
//            m_prevsp = GLFWStuff::sp2;
//        } else {
//            std::ofstream myFile("h_log.csv");
//            myFile << "inner_step";
//            myFile << ',';
//            myFile << "user_sp";
//            myFile << ',';
//            myFile << "cart_vel";
//            myFile << ',';
//            myFile << "cart_act";
//            myFile << ',';
//            myFile << "pitch";
//            myFile << ',';
//            myFile << "torque";
//            myFile << ',';
//            myFile << "step";
//            myFile << '\n';
//            for(int i=0; i<m_log_size; ++i){
//                myFile << m_log.col(i)[0];
//                myFile << ',';
//                myFile << m_log.col(i)[1];
//                myFile << ',';
//                myFile << m_log.col(i)[2];
//                myFile << ',';
//                myFile << m_log.col(i)[3];
//                myFile << ',';
//                myFile << m_log.col(i)[4];
//                myFile << ',';
//                myFile << m_log.col(i)[5];
//                myFile << ',';
//                myFile << m_log.col(i)[6];
//                myFile << '\n';
//            }
//            myFile.close();
//            SHOW("done")
//        }
    
//    if(m_total_subs < m_log_size){
//        m_log.col(m_total_subs)[0] = m_total_subs;
//        m_log.col(m_total_subs)[1] = GLFWStuff::sp2;
//        m_log.col(m_total_subs)[2] = m_prevsp-m_ob[0];
//        m_log.col(m_total_subs)[3] = m_act[0];
//        m_log.col(m_total_subs)[4] = d->sensordata[0];
//        m_log.col(m_total_subs)[5] = priNet->m_out[0];
//        m_log.col(m_total_subs)[6] = step;
//        m_total_subs += 1;
//        m_prevsp = GLFWStuff::sp2;
//    } else {
//        std::ofstream myFile("h_log.csv");
//        myFile << "inner_step";
//        myFile << ',';
//        myFile << "user_sp";
//        myFile << ',';
//        myFile << "cart_vel";
//        myFile << ',';
//        myFile << "cart_act";
//        myFile << ',';
//        myFile << "pitch";
//        myFile << ',';
//        myFile << "torque";
//        myFile << ',';
//        myFile << "step";
//        myFile << '\n';
//        for(int i=0; i<m_log_size; ++i){
//            myFile << m_log.col(i)[0];
//            myFile << ',';
//            myFile << m_log.col(i)[1];
//            myFile << ',';
//            myFile << m_log.col(i)[2];
//            myFile << ',';
//            myFile << m_log.col(i)[3];
//            myFile << ',';
//            myFile << m_log.col(i)[4];
//            myFile << ',';
//            myFile << m_log.col(i)[5];
//            myFile << ',';
//            myFile << m_log.col(i)[6];
//            myFile << '\n';
//        }
//        myFile.close();
//        SHOW("done")
        
    }
    step +=1;
    GetOb(); // Get next observation, rewards and done
}


void CartPoleEnvH::GetOverlay(){
    
    std::string det_string;
    if (m_deterministic) {
        det_string = "Deterministic";
    } else {
        det_string = "Stocastic";
    }
    
    label =  "Torque SP: " + std::to_string(d->ctrl[0]) + "Pitch SP: " + std::to_string(d->sensordata[0]) +
    "\nOb:P" + std::to_string(m_ob[0])+  " I" + std::to_string(m_ob[1]) +
    "\nRew: " + std::to_string(m_reward) +
    "\nFR " + std::to_string(int(GLFWStuff::framerate)) +
    "  " + std::to_string(int(1./(m->opt.timestep*n_sub_steps*m_rp))) + "Hz  " + std::to_string(int(d->time)) + "s"
    "\nStep " +  std::to_string(step) +
    "\n" + det_string +
    "\nSet_Point" + std::to_string(GLFWStuff::sp2) +
    "\nPitch " + std::to_string(d->sensordata[0]);
}


void CartPoleEnvH::LoadSubPolicy(const char * resourcePath, const char * jsonFile, int iter){
    
    // Find max iteration
    char iFile_sub[100] = "info_";
    strcat(iFile_sub, jsonFile);
    SHOW(iFile_sub)
    std::ifstream i_sub(iFile_sub);
    nlohmann::json j_sub;
    i_sub >> j_sub;
    int n_iter = int(j_sub.size())-1;
    SHOW2("iter", n_iter)

    std::string pFile_iter =std::to_string(n_iter) + "_policy_primary.json";
    std::string pFile = "policy_primary.json";
    
    std::ifstream i(pFile_iter);
    SHOW(pFile_iter)
    nlohmann::json j;
    i >> j;
    
    SHOW2(j[0]["W1"][0].size(), j[0]["W1"].size())
    priNet = new NeuralNetwork(j[0]["W1"][0].size(),
                               j[0]["W3"].size(),
                               j[0]["W1"].size(),
                               j[0]["W2"].size()
                               );
    
    
    
    priNet->LoadNetwork(resourcePath, pFile.c_str(), n_iter);
    
    
    
    
    SHOW2("\nMy Sub json file:\t",pFile)
    SHOW2("sub iter", n_iter)
    SHOW2("sub ob_size:\t", j[0]["W1"][0].size())
    SHOW2("sub h1_size:\t", j[0]["W1"].size())
    SHOW2("sub h2_size:\t", j[0]["W2"].size())
    SHOW2("sub out_size:\t", j[0]["W3"].size())
    
    
}


Eigen::VectorXf CartPoleEnvH::NextOb() {
    return m_ob;
}

uint16_t CartPoleEnvH::GetObSize() {
    return m_ob.size();
}

uint16_t CartPoleEnvH::GetActSize() {
    return m_act_size;
}

CartPoleEnvH::~CartPoleEnvH()
{
    SHOW("CartPoleEnvH destroyed");
}
