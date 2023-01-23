//
//  EnvTrolley.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 13/05/2022.
//

#include "EnvTrolley.hpp"


TrolleyEnv::TrolleyEnv(std::string name, const char * portName, int m_seed, int step_limit)
    :RealEnv(name, step_limit, portName)
{
    m_kill_angle = 0.65;
    m_ob = Eigen::VectorXf(m_ob_size);  //ob size
    m_act = Eigen::VectorXf(m_act_size);  //act size
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
    m_ob = Eigen::VectorXf(m_ob_size);  // ob size
    ConnectAgentSerial();
}


void TrolleyEnv::GetOb()
{
}

void TrolleyEnv::MarkovStep(Eigen::VectorXf agent_act)
{
    m_pitch_set_point = GLFWStuff::sp;
    // Convert to bytes array
    m_act = agent_act;
    u_int8_t serialOb[6];
    u_int8_t serialAct[3];
    serialAct[0] = u_int8_t(fmin(fmax(m_act[0]*100, -127), 127)+127);
    serialAct[1] = u_int8_t(fmin(fmax(m_act[0]*100, -127), 127)+127);
    serialAct[2] = u_int8_t(7);

    //Step
    write(serial_port, serialAct, sizeof(serialAct));
    const u_int8_t num_bytes = u_int8_t(read(serial_port, &serialOb, sizeof(serialOb)));
    
    m_now = std::chrono::steady_clock::now();
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(m_now - m_prev).count();
    int step_time_ms = int(value);
    m_prev = m_now;
    
    bool err = false;
    if (num_bytes != sizeof(serialOb)) {
        err = true;
        SHOW2("Lost packet, please wait for reboot ", int(num_bytes))
    } else {
        float main = float(serialOb[0])-100.0f;
        float dec = float((serialOb[1])/100.0f)-1.0f;
        m_true = (main + dec)/30.0f;
        
        float t =  step_time_ms/1000.0f;  //1.0f/.0005f;
        SHOW(step_time_ms);
        float err = m_true - m_pitch_set_point;
        
        m_P = err;
        m_D = ((err - m_prev_err) / t)/10.0;
        m_I = (err + m_prev_err) * t * 0.5 + m_prevI;
//        m_I = fmin(fmax(m_I, -3.65), 3.65);
        
        m_prev_err = err;
        m_prevI = m_I;
        m_prevD = m_D;
        
        m_ob[0] = m_P;
        m_ob[1] = m_I;
//        m_ob[2] = m_D;
        
        m_reward =(m_kill_angle-abs(err))*0.01;
//        SHOW2("ob", m_ob);
    }
    
    // Done ------------------------------------------------------------
//    m_kill_angle = 1.0;
    m_done = 0;
    if (err) {
        m_done = 3;
        SHOW("Done: Robot error")
    }else if (abs(m_ob[0]) > m_kill_angle and StepLimited) {
        m_done = 1;
//        m_reward = -1.;
        SHOW("Done: Fell over")
    }else if (step >= StepLimit-1 and StepLimited){
        m_done = 2;
        SHOW("Done: Step Limit Reached")
    }
//    SHOW("\n------------------------------------------")
//    SHOW2("Steps\t", step)
//    SHOW2("act\t", m_act[0])
//    std::cout << m_ob[0] << '\t' << m_ob[1] << std::endl;
//    SHOW2("Rew\t", m_reward)
    
    if (m_done){
        //stop robot
        SHOW("Done stopping")
        u_int8_t serialAct[3] = {127, 127, 1};
        write(serial_port, serialAct, sizeof(serialAct));
        const u_int8_t num_bytes = u_int8_t(read(serial_port, &serialOb, sizeof(serialOb)));
        if (num_bytes != sizeof(serialOb)) {
            SHOW("it crowd reset")
        }
    }
    if(!err){
        step +=1;
    }
    
//    m_now = std::chrono::steady_clock::now();
//    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(m_now - m_prev).count();
//    int step_time_ms = int(value);
//    m_prev = m_now;
    
    std::string det_string;
    if (m_deterministic) {
        det_string = "Deterministic";
    } else {
        det_string = "Stocastic";
    }
//    label =  "Act: " + std::to_string(m_act[0]) +
//    "\nOb: " + std::to_string(m_ob[0])+  " " + std::to_string(m_ob[1]) +
//    "\nRew: " + std::to_string(m_reward) +
//    "\nPeriod(ms) " + std::to_string(step_time_ms) +
//    "\nStep " +  std::to_string(step) +
//    "\n" + det_string +
//    "\nSet_Point1:\t" + std::to_string(m_pitch_set_point) +
//    "\nSet_Point2:\t" + std::to_string(GLFWStuff::sp2);
    label = "a";
//    Render();
//    glfwPollEvents();

}

void TrolleyEnv::EpisodeReset(int startPos) {
    m_ob = Eigen::VectorXf(m_ob_size);  //ob size
    m_P = 0;
    m_I = 0;
    m_D = 0;
    m_prev_err = 0;
    m_prevI = 0;
    m_prevD = 0;
    m_ob = Eigen::VectorXf(m_ob_size);  //ob size
    u_int8_t serialOb[6];
    u_int8_t serialAct[3] = {127, 127, 233};
    serialOb[0] = 200;

//    while (serialOb[0]!=101 and serialOb[0]!=99) {
    while (false) {
        m_pitch_set_point = GLFWStuff::sp;
        float main = float(serialOb[0])-100.0f;
        float dec = float((serialOb[1])/100.0f)-1.0f;
        SHOW2("angle:",(main + dec)/100.0f);
        
        label = "orientate:\t" + std::to_string(serialOb[0]);

        write(serial_port, serialAct, sizeof(serialAct));
        u_int8_t num_bytes = u_int8_t(read(serial_port, &serialOb, sizeof(serialOb)));
        Render();
        if (m_quit){break;}

        while (num_bytes == 255) {
            label = "Error: Serial needs resetting reboot app";
            Render();
            if (m_quit){break;}
        }
        while (num_bytes == 0) {
            write(serial_port, serialAct, sizeof(serialAct));
            num_bytes = u_int8_t(read(serial_port, &serialOb, sizeof(serialOb)));
            label = "Wait for robot to reset";
            Render();
        }
    }
    float main = float(serialOb[0])-100.0f;
    float dec = float((serialOb[1])/100.0f)-1.0f;
    m_ob[0] = (main + dec)/30.0f;
    m_ob[1] = 0;
//    m_ob[2] = 0;
    m_prev = std::chrono::steady_clock::now();
    
    //Wait until in a safe spot
    
    m_done = 0;
    m_target_moved = true;
    step = -1;
}


Eigen::VectorXf TrolleyEnv::NextOb() {
    return m_ob;
}

uint16_t TrolleyEnv::GetObSize() {
    return m_ob.size();
}

uint16_t TrolleyEnv::GetActSize() {
    return m_act_size;
}

TrolleyEnv::~TrolleyEnv()
{
    u_int8_t serialAct[3] = {127, 127, 233};
    write(serial_port, serialAct, sizeof(serialAct));
    
//    DisconnetAgentSerial();
    SHOW("Trolley destroyed");
}

void TrolleyEnv::DisconnetAgentSerial() {
    close(serial_port);
}
