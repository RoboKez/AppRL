//
//  EnvDragger.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 07/04/2022.
//

#ifndef EnvDragger_hpp
#define EnvDragger_hpp

#include  "Env.hpp"

class DraggerEnv:public MjEnv
{
public:
    std::string m_name = "dragger";
    Eigen::VectorXf m_ob = Eigen::VectorXf(4);  //ob size
    std::uint16_t m_act_size = 2;               // act size
    
    DraggerEnv(std::string name, int m_seed=0, int step_limit=200);
    void EpisodeReset(int startPos=0);
    void GetOb();
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
    ~DraggerEnv();
    
protected:
    virtual void GetOverlay();
    void TargetReset();
    std::default_random_engine m_generator;
    float m_position = 0;
};



#endif /* EnvDragger_hpp */
