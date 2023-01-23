//
//  EnvCartPoleMoCap.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 09/07/2022.
//

#ifndef EnvCartPoleMoCap_hpp
#define EnvCartPoleMoCap_hpp

#include  "Env.hpp"

class CartPoleMoCap:public MjEnv
{
public:
    std::string m_env_mode;
    std::uint16_t m_act_size = 1;
    std::uint16_t m_ob_size = 4;
    CartPoleMoCap(std::string name, int m_seed=0, int step_limit=200, std::string env_mode="cartpoleMDP");
    void EpisodeReset(int startPos=0);
    void GetOb();
    void MoCapStep(float pitch, float pos);
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
    ~CartPoleMoCap();
    float m_mocap_pitch = 0;
    float m_mocap_position = 0;
    
protected:
    virtual void GetOverlay();
    std::default_random_engine m_generator;
    
private:
    float m_set_point = 0;
};

#endif /* EnvCartPoleMoCap_hpp */
