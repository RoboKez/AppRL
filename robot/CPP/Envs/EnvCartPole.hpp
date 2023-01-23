//
//  EnvCartPole.hpp
//  RL_Bytes
//
//  Created by Kez Smithson Whitehead on 14/03/2022.
//

#ifndef EnvCartPole_hpp
#define EnvCartPole_hpp

#include  "Env.hpp"

class CartPoleEnv:public MjEnv
{
public:
    std::string m_ob_name;
    std::uint16_t m_ob_mode = 1; //0=PD, 1=P, 2=PI, 3=PID, 
    std::uint16_t m_act_size;
    std::uint16_t m_ob_size;
    CartPoleEnv(std::string name, int m_seed=0, int step_limit=200, int ob_mode=1);
    ~CartPoleEnv();
    void EpisodeReset(int startPos=0);
    void GetOb();
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
private:
    virtual void GetOverlay();
    std::default_random_engine m_generator;
    PID pidPole = NULL;
    PID pidCart = NULL;
    int m_ID = 1;
    float m_final_pitch = -0.65;
};

#endif /* EnvCartPole_hpp */
