//
//  EnvTrolley.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 13/05/2022.
//

#ifndef EnvTrolley_hpp
#define EnvTrolley_hpp

#include  "Env.hpp"

#include <stdio.h>

class TrolleyEnv:public RealEnv
{
public:
    std::uint16_t m_act_size = 1;               // act size
    std::uint16_t m_ob_size = 2;                // ob size
    
    TrolleyEnv(std::string name, const char * portName, int m_seed=0, int step_limit=200);
    void DisconnetAgentSerial();
    void EpisodeReset(int startPos=0);
    void GetOb();
    void MarkovStep(Eigen::VectorXf agent_act);
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
    ~TrolleyEnv();
    std::chrono::steady_clock::time_point m_now;
    std::chrono::steady_clock::time_point m_prev;
    u_int8_t packetID;
    
protected:
    std::string m_name = "trolley";
private:
    bool m_target_moved = true;
    float m_old_pole_pos;
    
    float m_P = 0;
    float m_I = 0;
    float m_D = 0;
    float m_prev_err = 0;
    float m_prevI = 0;
    float m_prevD = 0;
    float m_pitch_set_point = 0;
};

#endif /* EnvTrolley_hpp */
