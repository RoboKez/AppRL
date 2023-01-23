//
//  EnvCheetah.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 29/04/2022.
//

#ifndef EnvCheetah_hpp
#define EnvCheetah_hpp
#include  "Env.hpp"

//class CheetahEnv:public MjEnv
//{
//public:
//    Eigen::VectorXf m_ob = Eigen::VectorXf(17);  // ob size
//    std::uint16_t m_act_size = 6;                // act size
//    std::uint16_t m_ob_size = 17;
//    CheetahEnv(std::string name, int m_seed=0, int step_limit=200);
//    void EpisodeReset(int startPos=0);
//    void GetOb();
//    Eigen::VectorXf NextOb();
//    uint16_t GetObSize();
//    uint16_t GetActSize();
//    ~CheetahEnv();
//
//protected:
//    std::string m_name = "cheetah_baseline";
//    virtual void GetOverlay();
//    std::default_random_engine m_generator;
//private:
//    float xposbefore;
//    std::default_random_engine m_noise_generator;
//};


class CheetahEnv:public MjEnv
{
public:
    std::string m_ob_name;
    std::uint16_t m_act_size;
    std::uint16_t m_ob_size;
    CheetahEnv(std::string name, int m_seed=0, int step_limit=200);
    ~CheetahEnv();
    void EpisodeReset(int startPos=0);
    void GetOb();
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
private:
    virtual void GetOverlay();
    float xposbefore;
    std::default_random_engine m_generator;
};

#endif /* EnvCheetah_hpp */
