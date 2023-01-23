//
//  NN.cpp
//  robot
//
//  Created by Kez Smithson Whitehead on 20/06/2022.
//

#include "NN.hpp"

NeuralNetwork::NeuralNetwork(std::uint16_t in_size,
                             std::uint16_t out_size,
                             std::uint16_t h1_size,
                             std::uint16_t h2_size,
                             float lr,
                             std::uint16_t seed,
                             std::uint16_t minibatch_size,
                             bool adam,
                             bool ascent)
:m_in_size(in_size),
m_out_size(out_size),
m_h1_size(h1_size),
m_h2_size(h2_size),
m_lr(lr),
m_seed(seed),
m_minibatch_size(minibatch_size),
m_adam(adam),
m_ascent(ascent)
{
    SHOW("NN constructed")
    WeightInit();
    ZeroAdamMomentums();
};


void NeuralNetwork::WeightInit(float mu, float stand_dev, const float initial_bias){
    /*
     Builds a Gaussian Distribution with mu and stand_dev
     Generates weights from distribution
     Adds Xavier initialisation as the activsation function is tanh
     Sets bias to inital bias
     */
    
    /// Set generator and seed
    std::default_random_engine m_generator;
    m_generator.seed(m_seed);
    std::normal_distribution<double> distribution(mu, stand_dev);
    
    /// Generate Weights
    for(std::uint16_t i=0; i<m_W1.size(); ++i){m_W1(i) = distribution(m_generator);}
    for(std::uint16_t i=0; i<m_W2.size(); ++i){m_W2(i) = distribution(m_generator);}
    for(std::uint16_t i=0; i<m_W3.size(); ++i){m_W3(i) = distribution(m_generator);}
    
    /// Xavier Init on Weights as the NN uses Tanh activation function
    m_W1 *= sqrt(double(1)/double(m_in_size));
    m_W2 *= sqrt(double(1)/double(m_h1_size));
    m_W3 *= sqrt(double(1)/double(m_h2_size));
    
    /// Init Biases
    m_B1.setConstant(initial_bias);
    m_B2.setConstant(initial_bias);
    m_B3.setConstant(initial_bias);
}

void NeuralNetwork::ForwardPropagate(){
    /*
     Forward Propagate a single sample
    */
    m_Z1 = (m_W1 * m_in) + m_B1;    /// dot product then element addition
    TanhActivation(m_Z1, m_A1);     /// tanh
    m_Z2 = m_W2 * m_A1 + m_B2;      /// dot product then element addition
    TanhActivation(m_Z2, m_A2);     /// tanh
    m_Z3 = m_W3 * m_A2 + m_B3;      /// dot product then element addition
    m_A3 = m_Z3;                    /// Linear activation only. m_A3 is also y_hat
    m_out = m_A3;                   /// means of each dimention
}

void NeuralNetwork::ForwardPropagateMulti(Eigen::MatrixXf& mini_in){
    /*
     Forward Propagate multiple samples, i.e. minibatch or batch
    */
   
    // Layer 1
    m_Z1b = (m_W1 * mini_in);                                 /// dot product
    for(int i=0; i<m_Z1b.cols(); ++i){m_Z1b.col(i) += m_B1;}  /// element wise bais addition
    TanhActivationMulti(m_Z1b, m_A1b);                        /// activation
    
    // Layer 2
    m_Z2b = m_W2 * m_A1b;                                     /// dot product
    for(int i=0; i<m_Z2b.cols(); ++i){m_Z2b.col(i) += m_B2;}  /// element wise bais addition
    TanhActivationMulti(m_Z2b, m_A2b);                        /// activation
    
    // Layer 3
    m_Z3b = m_W3 * m_A2b;                                     /// dot product
    for(int i=0; i<m_Z3b.cols(); ++i){m_Z3b.col(i) += m_B3;}  /// element wise bais addition
    m_A3b = m_Z3b;                                            /// no activation, TODO conside replace with linear
}

void NeuralNetwork::BackPropagateMulti(Eigen::MatrixXf& label,
                                       Eigen::MatrixXf& mini_in,
                                       Eigen::VectorXf ppo_corrector){
    
    /*
     Back Propagate multiple samples, i.e. minibatch or batch
     ppo_corrector only for PPO reinforcement learning, else leave blank
    */
    const int mini_size = int(label.cols());                    /// samples in update
//    ForwardPropagateMulti(mini_in);                             /// update Zs and As
    
    if(ppo_corrector.size()==1 && ppo_corrector[0]==1){         /// MSE loss
        m_dZ3b = (m_A3b - label);
    } else {                                                    /// MSE loss with PPO correction, note if correction is 0 grads also 0
        m_dZ3b = (m_A3b - label);
        for (int i=0; i<mini_size; i++)                             /// update grads (TODO Parrel GPU)
        {
            m_dZ3b.col(i) = (m_A3b.col(i) - label.col(i)) * ppo_corrector(i);
        }
    }

    m_dA2b = (m_W3.transpose() * m_dZ3b);                       /// dot product
    m_dZ2b = m_dA2b.cwiseProduct(TanhDerivativeMulti(m_Z2b));   /// element wise multiplication
    
    m_dA1b = m_W2.transpose() * m_dZ2b;                         /// dot product
    m_dZ1b = m_dA1b.cwiseProduct(TanhDerivativeMulti(m_Z1b));   /// element wise multiplication
    
    m_dW1.setZero(m_h1_size, m_in_size);                        /// Zero grads
    m_dW2.setZero(m_h2_size, m_h1_size);
    m_dW3.setZero(m_out_size, m_h2_size);
    m_dB1.setZero(m_h1_size);
    m_dB2.setZero(m_h2_size);
    m_dB3.setZero(m_out_size);
    
    
    for (int i=0; i<mini_size; i++)                             /// update grads (TODO Parrel GPU)
    {
        m_dB3 += m_dZ3b.col(i);
        m_dB2 += m_dZ2b.col(i);
        m_dB1 += m_dZ1b.col(i);
        
        m_dW3 += m_dZ3b.col(i) * m_A2b.col(i).transpose();
        m_dW2 += m_dZ2b.col(i) * m_A1b.col(i).transpose();
        m_dW1 += m_dZ1b.col(i) * mini_in.col(i).transpose();
    }
    m_dB3 /= mini_size;                                         /// average grads
    m_dB2 /= mini_size;
    m_dB1 /= mini_size;
    m_dW3 /= mini_size;
    m_dW2 /= mini_size;
    m_dW1 /= mini_size;
    
    (m_adam) ? AdamUpdateMulti() : VanillaUpdateMulti();       /// update network
    m_loss += (m_A3b-label).array().pow(2).sum();              /// calculated loss just for logging
}


void NeuralNetwork::TanhActivation(Eigen::VectorXf& Z, Eigen::VectorXf& A){
    
    /// Clip to stop infs
    float clip_limit = 7.6;
    for(std::uint16_t i=0; i<Z.size(); ++i){
        if(Z(i)>clip_limit){
            Z(i)=clip_limit;
        }else if (Z(i)<-clip_limit){
            Z(i)= -clip_limit;
        }

        /// hipster tanh element wise
        A(i) = (exp(Z(i)) - exp(-Z(i))) / (exp(Z(i)) + exp(-Z(i)));
    }
}

void NeuralNetwork::TanhActivationMulti(Eigen::MatrixXf& Z, Eigen::MatrixXf& A){
    A = Eigen::MatrixXf(Z.rows(), Z.cols());
    
    /// Clip to stop infs
    float clip_limit = 7.6;
    for(std::uint16_t i=0; i<Z.size(); ++i){
        if(Z(i)>clip_limit){
            Z(i)=clip_limit;
        }else if (Z(i)<-clip_limit){
            Z(i)= -clip_limit;
        }

        /// hipster tanh element wise
        A(i) = (exp(Z(i)) - exp(-Z(i))) / (exp(Z(i)) + exp(-Z(i)));
    }
}

Eigen::MatrixXf NeuralNetwork::TanhDerivativeMulti(Eigen::MatrixXf& Z) {
    Eigen::MatrixXf AA = Eigen::MatrixXf(Z.rows(), Z.cols());
    /// Clip to stop infs
    for(std::uint16_t i=0; i<Z.size(); ++i){
        float clip_limit = 7.6;
        
        if(Z(i)>clip_limit){
            Z(i)=clip_limit;
        }else if (Z(i)<-clip_limit){
            Z(i)= -clip_limit;
        }
        // hipster tanh element wise
        AA(i) = 1.0f - pow((exp(Z(i)) - exp(-Z(i))) / (exp(Z(i)) + exp(-Z(i))), 2.0f);
    }
    return AA;
}

void NeuralNetwork::VanillaUpdateMulti(){
    float direction = -1;
    if(m_ascent){direction *= -1;}
    
    m_W1 = m_W1.array() + direction * m_lr * m_dW1.array();
    m_W2 = m_W2.array() + direction * m_lr * m_dW2.array();
    m_W3 = m_W3.array() + direction * m_lr * m_dW3.array();
    
    m_B1 = m_B1.array() + direction * m_lr * m_dB1.array();
    m_B2 = m_B2.array() + direction * m_lr * m_dB2.array();
    m_B3 = m_B3.array() + direction * m_lr * m_dB3.array();
}

void NeuralNetwork::AdamUpdateMulti(
                               float b1,
                               float b2,
                               float e_small){
    
    float direction = -1;
    if(m_ascent){direction *= -1;}
    
    m_dW1_mom = b1 * m_dW1_mom.array() + (1.0f - b1) * m_dW1.array();
    m_dW2_mom = b1 * m_dW2_mom.array() + (1.0f - b1) * m_dW2.array();
    m_dW3_mom = b1 * m_dW3_mom.array() + (1.0f - b1) * m_dW3.array();
    m_dB1_mom = b1 * m_dB1_mom.array() + (1.0f - b1) * m_dB1.array();
    m_dB2_mom = b1 * m_dB2_mom.array() + (1.0f - b1) * m_dB2.array();
    m_dB3_mom = b1 * m_dB3_mom.array() + (1.0f - b1) * m_dB3.array();
    
    m_dW1_mom2 = b2 * m_dW1_mom2.array() + (1.0f - b2) * m_dW1.array().pow(2);
    m_dW2_mom2 = b2 * m_dW2_mom2.array() + (1.0f - b2) * m_dW2.array().pow(2);
    m_dW3_mom2 = b2 * m_dW3_mom2.array() + (1.0f - b2) * m_dW3.array().pow(2);
    m_dB1_mom2 = b2 * m_dB1_mom2.array() + (1.0f - b2) * m_dB1.array().pow(2);
    m_dB2_mom2 = b2 * m_dB2_mom2.array() + (1.0f - b2) * m_dB2.array().pow(2);
    m_dB3_mom2 = b2 * m_dB3_mom2.array() + (1.0f - b2) * m_dB3.array().pow(2);

    
    m_W1 = m_W1.array() + direction * m_lr * m_dW1_mom.array() / (m_dW1_mom2.array().pow(0.5) + e_small);
    m_W2 = m_W2.array() + direction * m_lr * m_dW2_mom.array() / (m_dW2_mom2.array().pow(0.5) + e_small);
    m_W3 = m_W3.array() + direction * m_lr * m_dW3_mom.array() / (m_dW3_mom2.array().pow(0.5) + e_small);
    
    m_B1 = m_B1.array() + direction * m_lr * m_dB1_mom.array() / (m_dB1_mom2.array().pow(0.5) + e_small);
    m_B2 = m_B2.array() + direction * m_lr * m_dB2_mom.array() / (m_dB2_mom2.array().pow(0.5) + e_small);
    m_B3 = m_B3.array() + direction * m_lr * m_dB3_mom.array() / (m_dB3_mom2.array().pow(0.5) + e_small);
}


void NeuralNetwork::ZeroAdamMomentums(){
    
    m_dW1_mom.setZero(m_h1_size, m_in_size);
    m_dW2_mom.setZero(m_h2_size, m_h1_size);
    m_dW3_mom.setZero(m_out_size, m_h2_size);
    m_dB1_mom.setZero(m_h1_size);
    m_dB2_mom.setZero(m_h2_size);
    m_dB3_mom.setZero(m_out_size);
    
    m_dW1_mom2.setZero(m_h1_size, m_in_size);
    m_dW2_mom2.setZero(m_h2_size, m_h1_size);
    m_dW3_mom2.setZero(m_out_size, m_h2_size);
    m_dB1_mom2.setZero(m_h1_size);
    m_dB2_mom2.setZero(m_h2_size);
    m_dB3_mom2.setZero(m_out_size);
}

void NeuralNetwork::Train(std::uint16_t n_epochs){
    uint32_t n_samples = int(m_x_train.rows());
    
    /// Compute min_batch sizes
    std::uint16_t n_minibatches = int(ceil(n_samples / m_minibatch_size));
    std::uint16_t final_minibatch_size;
    if ((n_samples % m_minibatch_size) !=0) {
        final_minibatch_size = n_samples % m_minibatch_size;
        n_minibatches +=1;
    } else {
        final_minibatch_size = m_minibatch_size;
    }
    for(std::int16_t e=0; e < n_epochs; ++e){

        /// Shuffle batch_ids
        m_loss = 0;
        auto IDs = Eigen::VectorXi(n_samples);
        for (std::uint16_t i=0; i < n_samples; ++i){IDs[i] = i;}
        shuffle (IDs.begin(), IDs.end(), m_generator);
        for(std::int16_t m=0; m < n_minibatches; ++m){
            Eigen::VectorXi mini_IDs;
            std::uint16_t start;
            std::uint16_t end;
            if(m==(n_minibatches-1)){
                start = m * m_minibatch_size;
                end = m * m_minibatch_size + final_minibatch_size;
            } else {
                start = m * m_minibatch_size;
                end = m * m_minibatch_size + m_minibatch_size;
            }
            mini_IDs = IDs(Eigen::seqN(start,end-start));
            
            /// Make minibatch and update network
            Eigen::MatrixXf in_batch  = Eigen::MatrixXf(m_in_size, mini_IDs.size());
            Eigen::MatrixXf out_batch  = Eigen::MatrixXf(m_out_size, mini_IDs.size());
            for(std::int16_t i=0; i < mini_IDs.size(); ++i){
                std::uint16_t id = mini_IDs[i];
                in_batch.col(i)  = m_x_train.row(id);
                out_batch.col(i) = m_y_train.row(id);
            }
            BackPropagateMulti(out_batch, in_batch);
        }
        m_loss /= float(n_samples*m_out_size);
        SHOW2("Training loss\t", m_loss);
    }
};
  
float NeuralNetwork::Test(){
    float loss = 0;
    Eigen::MatrixXf in_batch  = m_x_test.transpose();
    Eigen::MatrixXf out_batch  = m_y_test.transpose();
    
    ForwardPropagateMulti(in_batch);
    loss += (m_A3b-out_batch).array().pow(2).sum()/(out_batch.size());  /// mse loss: change to out_batch.size() for error for  average each sample feature
    SHOW2("Model test loss\t", loss)
    return loss;
};

void NeuralNetwork::LoadDataset(const char * resourcePath,
                                const char * jsonFile,
                                int iter,
                                float split){
    
    std::string tmp_name = jsonFile;

    chdir(resourcePath);
    SHOW(resourcePath);
    SHOW(tmp_name);
    std::ifstream i(tmp_name);
    nlohmann::json jd;
    i >> jd;

    SHOW("Loading Dataset")
    SHOW2("resource path:\t", resourcePath)
    SHOW2("json file:\t", jsonFile)
    SHOW2("iter", iter)
    int n_samples = int(jd[0]["x"].size());
    int x_dim =int(jd[0]["x"][0].size());
    int y_dim = int(jd[0]["y"][0].size());
    
    int n_train = n_samples * split;
    SHOW2("n_train", n_train)
    int n_test = n_samples - n_train;
    m_x = Eigen::MatrixXf(n_samples, x_dim);
    m_y = Eigen::MatrixXf(n_samples, y_dim);

    for(uint16_t r=0; r < m_x.rows(); r++){for(uint16_t c=0; c < m_x.cols(); c++){m_x(r, c) = jd[0]["x"][r][c];}}
    for(uint16_t r=0; r < m_y.rows(); r++){for(uint16_t c=0; c < m_y.cols(); c++){m_y(r, c) = jd[0]["y"][r][c];}}

    // Randomly split train and test
    m_x_train = Eigen::MatrixXf(n_train, x_dim);
    m_y_train = Eigen::MatrixXf(n_train, y_dim);
    m_x_test = Eigen::MatrixXf(n_test, x_dim);
    m_y_test = Eigen::MatrixXf(n_test, y_dim);

    auto IDs = Eigen::VectorXi(n_samples);
    for (std::uint16_t i=0; i < n_samples; ++i){IDs[i] = i;}
    shuffle(IDs.begin(), IDs.end(), m_generator);

    for (std::uint16_t i=0; i < n_samples; ++i)
    {
        if(i<n_train){
            m_x_train.row(i) = m_x.row(IDs[i]);
            m_y_train.row(i) = m_y.row(IDs[i]);
        } else {
            m_x_test.row(i-n_train) = m_x.row(IDs[i]);
            m_y_test.row(i-n_train) = m_y.row(IDs[i]);
        }
    }

}

void NeuralNetwork::SaveNetwork(const char * resourcePath, const char * jsonFile, int iter){
    
    chdir(resourcePath);
    nlohmann::json jn;
    jn = nlohmann::json::parse(R"([])");
    std::string tmp_name = std::to_string(iter) + '_' + jsonFile;
    
    // Save network weights and biasies
    for(uint16_t i=0; i < m_W1.rows(); i++) { jn[0]["W1"][i] = m_W1.row(i); }
    for(uint16_t i=0; i < m_W2.rows(); i++) { jn[0]["W2"][i] = m_W2.row(i); }
    for(uint16_t i=0; i < m_W3.rows(); i++) { jn[0]["W3"][i] = m_W3.row(i); }
    jn[0]["B1"] = m_B1;
    jn[0]["B2"] = m_B2;
    jn[0]["B3"] = m_B3;
    SHOW2("saving... ", tmp_name)
    
    std::ofstream o(tmp_name);
    o << std::setw(4) << jn << std::endl;
    SHOW2("...saved as ", tmp_name)
    
}


void NeuralNetwork::LoadNetwork(const char * resourcePath, const char * jsonFile, int iter){
        
    chdir(resourcePath);
    std::string tmp_name = std::to_string(iter) + '_' + jsonFile;
    
    SHOW(tmp_name)
    std::ifstream i(tmp_name);
    nlohmann::json j;
    i >> j;

    SHOW("\nLoading NN iter")
    SHOW2("resource path:\t", resourcePath)
    SHOW2("jsob_file:\t", jsonFile)
    SHOW2("jsob_iter:\t", iter)
    SHOW2("in_size:\t", j[0]["W1"][0].size())
    SHOW2("h1_size:\t", j[0]["W1"].size())
    SHOW2("h2_size:\t", j[0]["W2"].size())
    SHOW2("out_size:\t", j[0]["W3"].size())

    m_W1 = Eigen::MatrixXf(int(j[0]["W1"].size()), int(j[0]["W1"][0].size()));
    m_W2 = Eigen::MatrixXf(int(j[0]["W2"].size()), int(j[0]["W2"][0].size()));
    m_W3 = Eigen::MatrixXf(int(j[0]["W3"].size()), int(j[0]["W3"][0].size()));

    m_B1 = Eigen::VectorXf(int(j[0]["B1"].size()));
    m_B2 = Eigen::VectorXf(int(j[0]["B2"].size()));
    m_B3 = Eigen::VectorXf(int(j[0]["B3"].size()));
    
    for(uint16_t r=0; r < m_W1.rows(); r++){for(uint16_t c=0; c < m_W1.cols(); c++){m_W1(r, c) = j[0]["W1"][r][c];}}
    for(uint16_t r=0; r < m_W2.rows(); r++){for(uint16_t c=0; c < m_W2.cols(); c++){m_W2(r, c) = j[0]["W2"][r][c];}}
    for(uint16_t r=0; r < m_W3.rows(); r++){for(uint16_t c=0; c < m_W3.cols(); c++){m_W3(r, c) = j[0]["W3"][r][c];}}

    for(uint16_t r=0; r < m_B1.size(); r++){m_B1(r) = j[0]["B1"][r];}
    for(uint16_t r=0; r < m_B2.size(); r++){m_B2(r) = j[0]["B2"][r];}
    for(uint16_t r=0; r < m_B3.size(); r++){m_B3(r) = j[0]["B3"][r];}
    
    m_in_size = j[0]["W1"][0].size();
    m_out_size = j[0]["W3"].size();
    m_h1_size  = j[0]["W1"].size();
    m_h2_size  = j[0]["W2"].size();
}

