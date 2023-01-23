//
//  EnvLinkNone.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 07/12/2022.
//

#ifndef EnvLinkNone_hpp
#define EnvLinkNone_hpp

#include  "Env.hpp"


class LinkNoneEnv:public MjEnv
{
public:
    std::uint16_t m_act_size = 8;
    std::uint16_t m_ob_size = 20;
    LinkNoneEnv(std::string name, int m_seed=0, int step_limit=200);
    ~LinkNoneEnv();
    
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
    std::string m_name = "LinkNone";
};

#endif /* EnvLinkNone_hpp */
