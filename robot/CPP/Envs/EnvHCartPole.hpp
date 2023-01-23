//
//  EnvHCartPole.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 09/06/2022.
//

#ifndef EnvHCartPole_hpp
#define EnvHCartPole_hpp

#include <json.hpp>
#include  "Env.hpp"
#include "NN.hpp"
class CartPoleEnvH:public MjEnv
{
public:
    std::uint16_t m_act_size = 1;               // act size
    std::uint16_t m_ob_size = 2;
    
    int m_log_size = 6000;
    float m_prevsp = 0;
    
    Eigen::MatrixXf m_log = Eigen::MatrixXf(7, m_log_size);
    int m_total_subs = 0;
    
    int m_ID = 1;
    
    PID pidPri = NULL;
    PID pidSec = NULL;
    
    CartPoleEnvH(std::string name, const char * resourcePath, const char * jsonFile, int sub_iter=-1, int m_seed=0, int step_limit=200);
    void LoadSubPolicy(const char * resourcePath, const char * jsonFile, int iter=-1);
    void EpisodeReset(int startPos=0) override;
    void GetOb() override;
    Eigen::VectorXf NextOb() override;
    uint16_t GetObSize() override;
    uint16_t GetActSize() override;
    ~CartPoleEnvH();
    
    virtual void MarkovStep(Eigen::VectorXf agent_act) override;  //override for hierachical

protected:
    std::string m_name = "cartpoleH";
    virtual void GetOverlay() override;
    void TargetReset(bool randomise=false);
    std::default_random_engine m_generator;
private:
    bool m_target_moved = true;
    float m_old_pole_pos;
    int m_push_counter = 0;
    float m_prev_wheel_vel = 0;
    
    float m_Psub = 0;
    float m_Isub = 0;
    float m_Dsub = 0;
    
    float m_prev_pitch_errsub = 0;
    float m_prevIsub = 0;
    float m_prevDsub = 0;
    
    float m_P = 0;
    float m_I = 0;
    float m_D = 0;
    
    float m_prev_err = 0;
    float m_prevI = 0;
    float m_prevD = 0;
    
    float m_prevTargetPitch = 0;
    
    float m_vel_set_point = 0;
    
//    NeuralNetwork *priPolicy = nullptr;
//    Agent *agent = nullptr;
    NeuralNetwork *priNet = nullptr;
    
    std::default_random_engine m_noise_generator;
};

#endif /* EnvHCartPole_hpp */
