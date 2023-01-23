//
//  EnvPiper.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 11/09/2022.
//

#ifndef EnvPiper_hpp
#define EnvPiper_hpp


#include  "Env.hpp"

class PiperEnv:public MjEnv
{
public:
    std::string m_ob_name;
    std::uint16_t m_ob_mode = 1; //0=PD, 1=P, 2=PI, 3=PID,
    std::uint16_t m_act_size;
    std::uint16_t m_ob_size;
    PiperEnv(std::string name, int m_seed=0, int step_limit=200, int ob_mode=1);
    ~PiperEnv();
    void EpisodeReset(int startPos=0);
    void GetOb();
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
    void MinLidar();
    
    float m_lidar_dist = 0;
    float m_lidar_ang = 0;
    float m_prev_lidar_dist = 0;
    float m_prev_lidar_ang = 0;
    float m_yaw = 0;
    
private:
    virtual void GetOverlay();
    std::default_random_engine m_generator;
    PID pidPole = NULL;
    PID pidCart = NULL;
    int m_ID = 5;
};


#endif /* EnvPiper_hpp */
