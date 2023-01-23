//
//  Agent.hpp
//  RL_Bytes_Swift
//
//  Created by Kez Smithson Whitehead on 28/03/2022.
//

#ifndef Agent_hpp
#define Agent_hpp
#include "NN.hpp"
#include <stdio.h>
#include <mujoco.h>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <random>

#include <json.hpp>
#include <iomanip>  //check if needed
#include <fstream>

#define SHOW(x) std::cout << x << std::endl;
#define SHOW2(x,y) std::cout << x << y << std::endl;

typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayXb;
class Agent {
public:
    bool m_first= false; 
    float m_running_act = 1.0;
//    Eigen::VectorXf m_running_act  = Eigen::VectorXf(m_act_size);// 0 to 1.0  where 1.0 is normal allows action smoothing 1.0=no smoothng
    float m_standard_deviation = 0.5;
    float m_min_standard_deviation = 0.3;
    float m_anneal_rate = 0.0003;
    float m_value_hat;
    std::default_random_engine m_generator;
    float m_lr;
    std::uint16_t m_seed;
    float m_log_prob;
    float m_log_prob_rev;
    bool m_load_network;
    bool m_adam;
    float m_sample_var = 0;
    int m_maas_mode = 0;
    std::uint16_t m_done_counter;

    
    // Actor architecture dimentions
    std::uint16_t m_in_size;
    std::uint16_t m_act_size;
    std::uint16_t m_h1_size;
    std::uint16_t m_h2_size;
    std::uint16_t m_desired_batch_size;
    std::uint16_t m_step_limit;
    std::uint16_t m_minibatch_size; //have user specified. must be less than m_batch_id
    Eigen::VectorXf m_model_hat = Eigen::VectorXf(m_in_size);
    
    std::uint16_t m_batch_id = 0; //
    float ip_clip = 0.2; //have user specified
    float m_avg_eps_reward_cumsum = 0;  //undiscounted
    
    NeuralNetwork mNet;
    NeuralNetwork vNet;
    NeuralNetwork pNet;
    // Actor mean output and action sample  =====================================================================================================
    Eigen::VectorXf m_mu  = Eigen::VectorXf(m_act_size);
    Eigen::VectorXf m_act = Eigen::VectorXf(m_act_size);
    Eigen::VectorXf m_prev_act = Eigen::VectorXf::Zero(m_act_size);
    Eigen::VectorXf m_ob = Eigen::VectorXf(m_in_size);
    
    
    
    // Cache from agent: mu, sigma, value_hat, log_likelihood(action_taken| mu, sigma), action_taken, done, model_hat ============================================
    std::uint16_t max_samples = m_desired_batch_size + m_step_limit + 1;  //big enough buffer, plus 1 for gae which isnt used
    Eigen::MatrixXf m_mu_cache = Eigen::MatrixXf(max_samples, m_act_size);
    Eigen::VectorXf m_standard_deviation_cache = Eigen::VectorXf(max_samples);
    Eigen::VectorXf m_value_hat_cache = Eigen::VectorXf(max_samples + 1);
    Eigen::VectorXf m_log_prob_cache = Eigen::VectorXf(max_samples);
    Eigen::VectorXf m_log_prob_rev_cache = Eigen::VectorXf(max_samples);
    Eigen::MatrixXf m_act_cache = Eigen::MatrixXf(max_samples, m_act_size);
    Eigen::MatrixXf m_model_hat_cache = Eigen::MatrixXf(max_samples, m_in_size);
    
    // Cache from environment: ob, rew, done. Note not next ob
    Eigen::MatrixXf m_ob_cache = Eigen::MatrixXf(max_samples, m_in_size);
    Eigen::MatrixXf m_next_ob_cache = Eigen::MatrixXf(max_samples, m_in_size);
    Eigen::VectorXf m_reward_cache = Eigen::VectorXf(max_samples);
    Eigen::VectorXi m_done_cache = Eigen::VectorXi(max_samples);
    
    // Cache generated from rewards and value estrimates
    Eigen::VectorXf m_return_cache = Eigen::VectorXf(max_samples);
    Eigen::VectorXf m_adv_cache = Eigen::VectorXf(max_samples);
    
    // Random Agent
    Agent(std::uint16_t in_size,
          std::uint16_t act_size,
          std::uint16_t h1_size = 4,
          std::uint16_t h2_size = 4,
          float lr = float(0.0003),
          std::uint16_t seed = std::chrono::system_clock::now().time_since_epoch().count(),
          std::uint16_t desired_batch_size=2000,
          std::uint16_t minibatch_size=64,
          std::uint16_t step_limit=200,
          bool load_network=false,
          bool adam=true);
    
    void AssignElements();
    
    void ForwardPropagate(Eigen::VectorXf env_ob);
    
    float GaussianLikelihood1D(float mu, float sigma, float x);
    
    bool CacheSample(float reward, int done, Eigen::VectorXf next_ob);
    
    void ClearSamples();
    
    void CalcLogProbRev(); //just for symmetry experiment
    
    void ReturnGAE(float gamma=0.99, float lambda=0.95);
    
    bool UpdatePPO(std::uint16_t n_epochs);
    
    void BackPropagate(Eigen::VectorXf ob, Eigen::VectorXf act, float ret, float adv, float old_log_prob, bool clipped, bool zero_grad_buffer);
    
    void ZeroAdamMomentums();  // move to private
    
    void LinearAnneal();
    
    void SaveModelData(const char * resourcePath, const char * jsonFile, const int iter);
    
    void SampleAction();
    
};

#endif /* Agent_hpp */
