//
//  EnvCartPoleMDP.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 17/06/2022.
//

#ifndef EnvCartPoleMDP_hpp
#define EnvCartPoleMDP_hpp

#include  "Env.hpp"

class CartPoleMDPEnv:public MjEnv
{
public:
    std::uint16_t m_act_size = 1;               // act size
    std::uint16_t m_ob_size = 4;                // ob size
    std::uint16_t m_mdp = 0;
    
    CartPoleMDPEnv(std::string name, int m_seed=0, int step_limit=200);
    void EpisodeReset(int startPos=0);
    void GetOb();
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
    ~CartPoleMDPEnv();
    
protected:
    std::string m_name = "cartpoleHVel";
    virtual void GetOverlay();
    void TargetReset(bool randomise=false);
    std::default_random_engine m_generator;
private:
    std::default_random_engine m_noise_generator;
    bool m_target_moved = true;
    float m_old_pole_pos;
    int m_push_counter = 0;
    float m_pitch_set_point = 0;
    
    float m_prev_pitch = 0;

};
#endif /* EnvCartPoleMDP_hpp */
