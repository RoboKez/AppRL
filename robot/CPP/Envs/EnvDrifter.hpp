//
//  EnvDrifter.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 15/07/2022.
//

#ifndef EnvDrifter_hpp
#define EnvDrifter_hpp

#include  "Env.hpp"

class DrifterEnv:public MjEnv
{
public:
    std::uint16_t m_act_size = 2;
    std::uint16_t m_ob_size = 4;
    DrifterEnv(std::string name, int m_seed=0, int step_limit=200);
    ~DrifterEnv();
    void EpisodeReset(int startPos=0);
    void GetOb();
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
private:
    int m_ID1 = 0;
    int m_ID2 = 1;
    virtual void GetOverlay();
    std::default_random_engine m_generator;
    PID pidX = NULL;
    PID pidY = NULL;
};

#endif /* EnvDrifter_hpp */
