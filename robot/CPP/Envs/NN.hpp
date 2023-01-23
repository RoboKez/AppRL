////
//  NN.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 20/06/2022.
//

#ifndef NN_hpp
#define NN_hpp

#include <stdio.h>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <random>
#include <json.hpp>
#include <iomanip>
#include <fstream>
#include <sys/param.h>
#include <unistd.h>

#define SHOW(x) std::cout << x << std::endl;
#define SHOW2(x,y) std::cout << x << y << std::endl;


typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayXb;

class NeuralNetwork {
public:
    std::uint16_t m_seed;
    std::default_random_engine m_generator;
    float m_lr;
    bool m_adam;
    bool m_load;
    bool m_ascent;
    
    // Actor architecture dimentions
    std::uint16_t m_in_size;
    std::uint16_t m_out_size;
    std::uint16_t m_h1_size;
    std::uint16_t m_h2_size;
    std::uint16_t m_batch_size;
    std::uint16_t m_minibatch_size;
    
    std::uint16_t m_buffer_count;
    Eigen::MatrixXf m_x;
    Eigen::MatrixXf m_y;
    Eigen::MatrixXf m_x_train;
    Eigen::MatrixXf m_y_train;
    Eigen::MatrixXf m_x_test;
    Eigen::MatrixXf m_y_test;
    
    Eigen::VectorXf m_in = Eigen::VectorXf(m_in_size);
    Eigen::VectorXf m_out  = Eigen::VectorXf(m_out_size);
    
    
    // Weights and Bias
    Eigen::MatrixXf m_W1 = Eigen::MatrixXf(m_h1_size, m_in_size);
    Eigen::MatrixXf m_W2 = Eigen::MatrixXf(m_h2_size, m_h1_size);
    Eigen::MatrixXf m_W3 = Eigen::MatrixXf(m_out_size, m_h2_size);
    Eigen::VectorXf m_B1 = Eigen::VectorXf(m_h1_size);
    Eigen::VectorXf m_B2 = Eigen::VectorXf(m_h2_size);
    Eigen::VectorXf m_B3 = Eigen::VectorXf(m_out_size);
    

    // Grads
    Eigen::MatrixXf m_dW1 = Eigen::MatrixXf(m_h1_size, m_in_size);
    Eigen::MatrixXf m_dW2 = Eigen::MatrixXf(m_h2_size, m_h1_size);
    Eigen::MatrixXf m_dW3 = Eigen::MatrixXf(m_out_size, m_h2_size);
    Eigen::VectorXf m_dB1 = Eigen::VectorXf(m_h1_size);
    Eigen::VectorXf m_dB2 = Eigen::VectorXf(m_h2_size);
    Eigen::VectorXf m_dB3 = Eigen::VectorXf(m_out_size);

    // Grad Buffer
    Eigen::MatrixXf m_dW1_buf = Eigen::MatrixXf(m_h1_size, m_in_size);
    Eigen::MatrixXf m_dW2_buf = Eigen::MatrixXf(m_h2_size, m_h1_size);
    Eigen::MatrixXf m_dW3_buf = Eigen::MatrixXf(m_out_size, m_h2_size);
    Eigen::VectorXf m_dB1_buf = Eigen::VectorXf(m_h1_size);
    Eigen::VectorXf m_dB2_buf = Eigen::VectorXf(m_h2_size);
    Eigen::VectorXf m_dB3_buf = Eigen::VectorXf(m_out_size);
    
    // Adam grad sums
    Eigen::MatrixXf m_dW1_mom;
    Eigen::MatrixXf m_dW2_mom;
    Eigen::MatrixXf m_dW3_mom;
    Eigen::VectorXf m_dB1_mom;
    Eigen::VectorXf m_dB2_mom;
    Eigen::VectorXf m_dB3_mom;
    
    Eigen::MatrixXf m_dW1_mom2;
    Eigen::MatrixXf m_dW2_mom2;
    Eigen::MatrixXf m_dW3_mom2;
    Eigen::VectorXf m_dB1_mom2;
    Eigen::VectorXf m_dB2_mom2;
    Eigen::VectorXf m_dB3_mom2;
    
    // Actor Pre activation
    Eigen::VectorXf m_Z1 = Eigen::VectorXf(m_h1_size);
    Eigen::VectorXf m_Z2 = Eigen::VectorXf(m_h2_size);
    Eigen::VectorXf m_Z3 = Eigen::VectorXf(m_out_size);
    
    Eigen::VectorXf m_dZ1 = Eigen::VectorXf(m_h1_size);
    Eigen::VectorXf m_dZ2 = Eigen::VectorXf(m_h2_size);
    Eigen::VectorXf m_dZ3 = Eigen::VectorXf(m_out_size);
    
    Eigen::MatrixXf m_Z1b;
    Eigen::MatrixXf m_Z2b;
    Eigen::MatrixXf m_Z3b;
    
    Eigen::MatrixXf m_dZ1b;
    Eigen::MatrixXf m_dZ2b;
    Eigen::MatrixXf m_dZ3b;
    
    // Actor Post activation
    Eigen::VectorXf m_A1 = Eigen::VectorXf(m_h1_size);
    Eigen::VectorXf m_A2 = Eigen::VectorXf(m_h2_size);
    Eigen::VectorXf m_A3 = Eigen::VectorXf(m_out_size);
    
    Eigen::VectorXf m_dA1 = Eigen::VectorXf(m_h1_size);
    Eigen::VectorXf m_dA2 = Eigen::VectorXf(m_h2_size);
    Eigen::VectorXf m_dA3 = Eigen::VectorXf(m_out_size);
    
    Eigen::MatrixXf m_A1b;
    Eigen::MatrixXf m_A2b;
    Eigen::MatrixXf m_A3b;
    
    Eigen::MatrixXf m_dA1b;
    Eigen::MatrixXf m_dA2b;
    Eigen::MatrixXf m_dA3b;
    
    
    float m_loss = 0;


    // New Initialiser
    NeuralNetwork(std::uint16_t in_size,
                  std::uint16_t out_size,
                  std::uint16_t h1_size = 64,
                  std::uint16_t h2_size = 64,
                  float lr = float(0.0003),
                  std::uint16_t seed = 1,
                  std::uint16_t minibatch_size = 64,
                  bool adam=true,
                  bool ascent=false);
    
    
    void WeightInit(float mu = float(0),
                    float stand_dev = float(1),
                    const float initial_bias=0.01);
    
    void LoadNetwork(const char * resourcePath,
                     const char * jsonFile,
                     int iter);
    
    void ForwardPropagate();
    void ForwardPropagateMulti(Eigen::MatrixXf& mini_in);
    
    void TanhActivation(Eigen::VectorXf& Z, Eigen::VectorXf& A);
    void TanhActivationMulti(Eigen::MatrixXf& Z, Eigen::MatrixXf& A);
    Eigen::MatrixXf TanhDerivativeMulti(Eigen::MatrixXf& Z);
    
    void BackPropagateMulti(Eigen::MatrixXf& label,
                            Eigen::MatrixXf& mini_in,
                            Eigen::VectorXf ppo_corrector = Eigen::VectorXf::Ones(1));
    
    void VanillaUpdateMulti();
    
    void ZeroAdamMomentums();
    
    void AdamUpdateMulti(
                    float b1=0.9,
                    float b2=0.999,
                    float e_small=float(1e-7));
    
    void Train(std::uint16_t n_epochs);
    float Test();
    void LoadDataset(const char * resourcePath,
                     const char * jsonFile,
                     int iter,
                     float split);
    
    void SaveNetwork(const char * resourcePath,
                     const char * jsonFile,
                     int iter);
};


#endif /* NN_hpp */


