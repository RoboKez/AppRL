//
//  EnvLidar.hpp
//  RL_Bytes
//
//  Created by Kez Smithson Whitehead on 14/03/2022.
//

#ifndef EnvLidar_hpp
#define EnvLidar_hpp

#include  "Env.hpp"

class LidarEnv:public MjEnv
{
public:
    std::uint16_t m_act_size = 2;
    std::uint16_t m_ob_size = 2;
    LidarEnv(std::string name, int m_seed=0, int step_limit=200);
    ~LidarEnv();
    
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
    
    void TargetReset();
    float prev_min_lidar = 0;
    bool target_moved = true;
    std::string m_name = "lidarbot";
};

#endif /* EnvLidar_hpp */
