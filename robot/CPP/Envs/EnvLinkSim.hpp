//
//  EnvLinkSim.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 04/01/2023.
//

#ifndef EnvLinkSim_hpp
#define EnvLinkSim_hpp

#include  "Env.hpp"


class LinkSimEnv:public MjEnv
{
public:
    std::uint16_t m_act_size = 4;
    std::uint16_t m_ob_size = 20;
    LinkSimEnv(std::string name, int m_seed=0, int step_limit=200);
    ~LinkSimEnv();
    
    void EpisodeReset(int startPos=0);
    void GetOb();
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
    virtual void MarkovStep(Eigen::VectorXf agent_act);

private:
    int m_ID1 = 0;
    int m_ID2 = 1;
    virtual void GetOverlay();
    std::default_random_engine m_generator;
    PID pidX = NULL;
    PID pidY = NULL;
    
    void TargetReset();
    float prev_min_lidar = 0;
    bool target_moved = true;
    std::string m_name = "link_symmetry";
};

#endif /* EnvLinkSim_hpp */
