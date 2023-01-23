//
//  EnvHHCartPole.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 28/06/2022.
//

#ifndef EnvHHCartPole_hpp
#define EnvHHCartPole_hpp

#include <json.hpp>
#include  "Env.hpp"
#include "NN.hpp"
class CartPoleEnvHH:public MjEnv
{
public:
    std::uint16_t m_act_size = 1;               // act size
    std::uint16_t m_ob_size = 2;                // ob size
    
    CartPoleEnvHH(std::string name,
                  const char * resourcePath,
                  const char * jsonFilePri,
                  const char * jsonFileSec,
                  int iterPri=-1,
                  int iterSec=-1,
                  int m_seed=0,
                  int step_limit=200);
    
    void EpisodeReset(int startPos=0) override;
    void GetOb() override;
    Eigen::VectorXf NextOb() override;
    uint16_t GetObSize() override;
    uint16_t GetActSize() override;
    ~CartPoleEnvHH();
    
    virtual void MarkovStep(Eigen::VectorXf agent_act) override;  //override for hierachical

protected:
    std::string m_name = "cartpoleH";
    virtual void GetOverlay() override;
    void TargetReset(bool randomise=false);
    std::default_random_engine m_generator;
private:
    void LoadSubPolicy(const char * resourcePath,
                       const char * jsonFilePri,
                       const char * jsonFileSec,
                       int iterPri = -1,
                       int iterSec = -1);
    void clampPID();
    void clampSubPID();
    bool m_target_moved = true;
    float m_old_pole_pos;
    int m_push_counter = 0;
//    float m_prev_wheel_vel = 0;
    
    
    float m_P_pri = 0;
    float m_I_pri = 0;
    float m_D_pri = 0;
    
    float m_prev_err_pri = 0;
    float m_prevI_pri = 0;
    float m_prevD_pri = 0;
    
    float m_P_sec = 0;
    float m_I_sec = 0;
    float m_D_sec = 0;
    
    float m_prev_err_sec = 0;
    float m_prevI_sec = 0;
    float m_prevD_sec = 0;
    
    float m_P_ter = 0;
    float m_I_ter = 0;
    float m_D_ter = 0;
    
    float m_prev_err_ter = 0;
    float m_prevI_ter = 0;
    float m_prevD_ter = 0;
    
    NeuralNetwork *priNet = nullptr;
    NeuralNetwork *secNet = nullptr;
    
    std::default_random_engine m_noise_generator;
};


#endif /* EnvHHCartPole_hpp */
