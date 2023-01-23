//
//  EnvCartPole.cpp
//  RL_Bytes
//
//  Created by Kez Smithson Whitehead on 14/03/2022.
//

#include "EnvCartPole.hpp"

CartPoleEnv::CartPoleEnv(std::string name, int m_seed, int step_limit, int ob_mode)
    : MjEnv(name, step_limit), m_ob_mode(ob_mode)

{
    n_sub_steps = 7;
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
        m_ob_size = 3;
    }
    else if(m_ob_mode==4)
    {
        m_ob_name = "ClassicI";
        m_ob_size = 5;
    }else if(m_ob_mode==5)
   {
        m_ob_name = "Classic";
        m_ob_size = 4;
    } else {
        SHOW("RL Error: Invalid ob mode")
    }
    SHOW2("Ob mode = ", m_ob_name)
    
    m_act_size = 1;
    m_generator.seed(m_seed);
    cam.distance = 5;
    m_ob = Eigen::VectorXf(m_ob_size);  // ob size
    pidPole = PID(m->opt.timestep*n_sub_steps);
    pidCart = PID(m->opt.timestep*n_sub_steps);
}

void CartPoleEnv::EpisodeReset(int startPos)
{
    
//    GLFWStuff::sp2 = 1.0;
    simstart = 0;
    mj_resetData(m, d);
//    d->mocap_pos[0] = GLFWStuff::sp2*2.0;
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    std::normal_distribution<> noise_dis(0.0, 0.01);

    
//    if (startPos==0){  // fixed with little noise
//        d->qpos[m_ID] = 0.0 + float(uni_dist(m_generator))/1000.0f*0.03f;
//    } else if (startPos==1){  // uniform random
//        d->qpos[m_ID] = 0.65f * m_last_side + float(uni_dist(m_generator))/1000.0f*0.03f;
//    } else if (startPos==2){  // fall position
//        d->qpos[m_ID] = -0.65f + float(uni_dist(m_generator))/1000.0f*0.03f;;
////        d->qpos[m_ID] = float(uni_dist(m_generator))/1000.0f*0.03f;
////        d->qpos[m_ID] = 0.65f;
////        d->qpos[m_ID] = float(uni_dist(m_generator))/1000.0f*0.03f;
//    }
    d->qpos[m_ID] = -0.65;
    SHOW2("\nreset angle ", d->qpos[m_ID])
    
    mj_forward(m, d);
    m_danger_zone_count = 0;
    step = -1;
    pidPole.Reset();
    pidCart.Reset();
    GetOb();
}
//3.5053587

void CartPoleEnv::GetOb()
{
    std::normal_distribution<> noise_dis(0.0, m_sensor_noise);
    std::uniform_int_distribution<> uni_dist(-1000, 1000);
    m_true = -d->sensordata[0]*rad_ratio/30.0f;//6 0.0f - noise_dis(m_generator);
    float trueCart = (d->sensordata[3]/2.0 + noise_dis(m_generator));
    pidPole.Update(m_true, GLFWStuff::sp);
    pidCart.Update(trueCart, GLFWStuff::sp2);
 
    if(m_ob_mode==0){
        m_ob[0] = pidPole.P;
        m_ob[1] = pidPole.D/5.0;
//        m_ob[1] = d->sensordata[2]/5.0;
    }
    else if(m_ob_mode==1)
    {
        m_ob[0] = pidPole.P;
    }
    else if(m_ob_mode==2)
    {
        m_ob[0] = pidPole.P;
        m_ob[1] = pidPole.I;
    }
    else if(m_ob_mode==3)
    {
        m_ob[0] = pidPole.P;
        m_ob[1] = pidPole.I;
        m_ob[2] = pidPole.D/5.0;
        
    } else if(m_ob_mode==4){
        m_ob[0] = pidPole.P;
        m_ob[1] = d->sensordata[1]/5.0;
        m_ob[2] = pidCart.P;
        m_ob[3] = d->sensordata[2]/5.0;
        m_ob[4] = pidCart.I;
        
    } else if(m_ob_mode==5){ //classic pvpv
        m_ob[0] = pidPole.P;
        m_ob[1] = d->sensordata[1]/5.0;
        m_ob[2] = pidCart.P;
        m_ob[3] = d->sensordata[2]/5.0;  // joint velocity
    
        
    } else {
        SHOW("RL Error: Invalid ob mode")
        SHOW(m_ob_mode)
    }
    
    
    
//    m_reward = ((1.0/(1.0 + abs(pidPole.P)))-0.5f) * 0.01;
//    m_reward = ((1.0/(1.0 + abs(pidPole.P)))) * 0.01;
    

    
    if(m_ob_mode > 3){
        m_ob[2]  = std::max(-m_kill_distance, std::min(m_ob[2] , m_kill_distance));
        m_reward = ((1.0/(1.0 + abs(pidPole.P)))) * 0.005 + ((1.0/(1.0 + abs(pidCart.P)))) * 0.005;
    } else {
        m_reward = ((1.0/(1.0 + abs(pidPole.P)))) * 0.01;
    }
    
    m_final_pitch = m_ob[0];
    if (m_final_pitch > m_kill_angle){
        m_final_pitch =0.65;
    } else if (m_final_pitch < -m_kill_angle){
        m_final_pitch =-0.65;
    }
    m_done = 0;
    
    if (abs(m_true) > m_kill_angle){
        m_done = 1;
        SHOW("fell over")
    } else if (step >= StepLimit-1){
        m_done = 2;
    }
    if (m_ob[0]<0){
        m_last_side = -1.0;
    } else {
        m_last_side = 1.0;
    }
//    if (m_last_side== 1.0){
//        m_last_side = -1.0;
//    } else {
//        m_last_side = 1.0;
//    }

    if (m_ob_mode > 3 && abs(m_ob[2])>=m_kill_distance){
        m_done = 1;
        SHOW("out of bounds")
//        if (m_final_pitch > 0){
//            m_final_pitch =0.65;
//        } else{
//            m_final_pitch =-0.65;
//        }
    }
}

void CartPoleEnv::GetOverlay(){
    
    std::string det_string;
    if (m_deterministic) {
        det_string = "Deterministic";
    } else {
        det_string = "Stocastic";
    }
    
    std::string ob_show = "\nOb: ";
    std::string sp_show = "\nSet_Point" + std::to_string(GLFWStuff::sp);
    
    if(m_ob_mode==0){ob_show = ob_show + "P:"  + std::to_string(m_ob[0]) + "D:" + std::to_string(m_ob[1]) + "tmp " + std::to_string(d->sensordata[2]/5.0);}
    else if(m_ob_mode==1){ob_show = ob_show + "P:"  + std::to_string(m_ob[0]);}
    else if(m_ob_mode==2){ob_show = ob_show + "P:"  + std::to_string(m_ob[0]) + "I:" + std::to_string(m_ob[1]);}
    else if(m_ob_mode==3){ob_show = ob_show + "P:"  + std::to_string(m_ob[0]) + "I:" + std::to_string(m_ob[1]) + "D:" + std::to_string(m_ob[2]);}
    else if(m_ob_mode==4){
        ob_show = ob_show + "PoleP:"  + std::to_string(m_ob[0]) + "PoleV:" + std::to_string(m_ob[1]) + "CartP:" + std::to_string(m_ob[2]) + "CartV:" + std::to_string(m_ob[3]) + "CartI:" + std::to_string(m_ob[4]);
        sp_show +=  ("\nSet_Point" + std::to_string(GLFWStuff::sp2));
    }
    
    else if(m_ob_mode==5){
        ob_show = ob_show + "PoleP:"  + std::to_string(m_ob[0]) + "PoleV:" + std::to_string(m_ob[1]) + "CartP:" + std::to_string(m_ob[2]) + "CartV:" + std::to_string(m_ob[3]);
        sp_show +=  ("\nSet_Point" + std::to_string(GLFWStuff::sp2));
    }
    else {SHOW("RL Error: Invalid ob mode")}
    d->mocap_pos[0] = GLFWStuff::sp2*2.0;
    
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

Eigen::VectorXf CartPoleEnv::NextOb() {
    return m_ob;
}

uint16_t CartPoleEnv::GetObSize() {
    return m_ob_size;
}

uint16_t CartPoleEnv::GetActSize() {
    return m_act_size;
}

CartPoleEnv::~CartPoleEnv()
{
    SHOW("CartPoleEnv destroyed");
}

