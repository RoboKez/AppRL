//
//  Agent.cpp
//  RL_Bytes_Swift
//
//  Created by Kez Smithson Whitehead on 28/03/2022.
//

#include "Agent.hpp"

// Create random agent
Agent::Agent(std::uint16_t in_size,
             std::uint16_t act_size,
             std::uint16_t h1_size,
             std::uint16_t h2_size,
             float lr,
             std::uint16_t seed,
             std::uint16_t desired_batch_size,
             std::uint16_t minibatch_size,
             std::uint16_t step_limit,
             bool load_network,
             bool adam)
:m_in_size(in_size),
m_act_size(act_size),
m_h1_size(h1_size),
m_h2_size(h2_size),
m_lr(lr),
m_seed(seed),
m_desired_batch_size(desired_batch_size),
m_minibatch_size(minibatch_size),
m_step_limit(step_limit),
m_load_network(load_network),
m_adam(adam),
vNet(
         m_in_size,
         1,
         m_h1_size,
         m_h2_size,
         m_lr,
         m_seed,
         m_minibatch_size,
         m_adam,
         false  //false is descent
         ),
mNet(
         m_act_size+m_in_size,
         m_in_size,
         m_h1_size,
         m_h2_size,
         m_lr,
         m_seed,
         m_minibatch_size,
         m_adam,
         false
         ),
pNet(
         m_in_size,
         m_act_size,
         m_h1_size,
         m_h2_size,
         m_lr,
         m_seed,
         m_minibatch_size,
         m_adam,
         true
         )
{
    SHOW2("Agent created: max_samples agent", max_samples)

    ZeroAdamMomentums();
};


void Agent::CalcLogProbRev(){
    
    // Builds an artifical data set that assumes symetry
    
    // Assumes symetery
    for(int i=0; i<m_batch_id; ++i){
        // Value  prediction -------------------------------------------------------------------
        vNet.m_in = -m_ob;  //NOTE MINUS!!!
        vNet.ForwardPropagate();
        
        // Action  mu and sampling ---------------------------------------------------------------
        pNet.m_in = -m_ob;  //NOTE MINUS!!!
        pNet.ForwardPropagate();
        m_mu = pNet.m_A3;
        m_value_hat = vNet.m_A3(0);
        
        // Assume took minus action action taken
        m_act = -m_act_cache.row(i); //NOTE MINUS!!!
        
        // Find gaussian log likelihood of MINUS m_act
        Eigen::VectorXf prob_breakdown = Eigen::VectorXf(m_act_size);
        for(std::uint16_t i=0; i<m_act_size; ++i){
          prob_breakdown(i) = GaussianLikelihood1D(m_mu(i), m_standard_deviation, m_act(i));
        }
        m_log_prob = log(prob_breakdown.prod());  //check base
        m_log_prob_rev_cache(i) = m_log_prob;
    }
}

void Agent::ForwardPropagate(Eigen::VectorXf env_ob){
    /// first is only to zero previous action for running average experiments.
    m_ob = env_ob;
    
    // Value Model prediction -------------------------------------------------------------------
    vNet.m_in = m_ob;
    vNet.ForwardPropagate();
    
    // Action Model mu and sampling ---------------------------------------------------------------
    pNet.m_in = m_ob;
    pNet.ForwardPropagate();
    m_mu = pNet.m_A3;
    m_value_hat = vNet.m_A3(0);
    
    // Sample each action dimension --------------------------------------------------------------
    // Assumption: standard deviation same for all dimentions
    SampleAction(); // sampling type dependant on m_mass_mode and m_filter set to 0 and 1 for vanilla sampling
    
    // Find gaussian log likelihood of m_act ---------------------------------------------------------
    Eigen::VectorXf prob_breakdown = Eigen::VectorXf(m_act_size);
    for(std::uint16_t i=0; i<m_act_size; ++i){
      prob_breakdown(i) = GaussianLikelihood1D(m_mu(i), m_standard_deviation, m_act(i));
    }
    m_log_prob = log(prob_breakdown.prod());  //check base
    
//    // Dynamics Model prediction -------------------------------------------------------------------
//    for(uint16_t ii=0; ii < m_in_size; ii++){mNet.m_in[ii]=m_ob[ii];}
//    for(uint16_t ii=m_in_size; ii < m_in_size+m_act_size; ii++){mNet.m_in[ii] = m_act[ii-m_in_size];}
//    mNet.ForwardPropagate();
}


float Agent::GaussianLikelihood1D(float mu, float sigma, float x){
    float likelihood = (exp(-0.5f*(pow(x-mu, 2))/(pow(sigma, 2))))/sqrt(M_PI*2.0f*pow(sigma, 2));
    return likelihood;
}


bool Agent::CacheSample(float reward, int done, Eigen::VectorXf next_ob){
    /*
     Saves sample set
    ----------------------------------------
     m_mu (vector of act_size),
     m_standard_deviation (scalar for now),
     m_value_hat predicted value_hat of observation (scalar),
     m_log_prob (scalar),
     m_act (vector of act size)
     m_done (scalar)
     */
    if(m_batch_id > max_samples-1) {
        SHOW("RL Error: Exceed sample buffer, this sample wasnt added")
        return true;
    } else {
        m_mu_cache.row(m_batch_id) = m_mu;
        m_standard_deviation_cache(m_batch_id) = m_standard_deviation;
        m_value_hat_cache(m_batch_id) = m_value_hat;
        m_log_prob_cache(m_batch_id) = m_log_prob;
        m_act_cache.row(m_batch_id) = m_act;
        m_done_cache(m_batch_id) = done;
        m_reward_cache(m_batch_id) = reward;
        m_ob_cache.row(m_batch_id) = m_ob;
        m_next_ob_cache.row(m_batch_id) = next_ob;
        
        m_batch_id +=1;  // ToDO add reset  //todo to big think addding one og null
        return false;
    }
}


void Agent::ReturnGAE(float gamma, float lambda){
    /*
     ref GAE paper: https//arxiv.org/pdf/1506.02438.pdf
     */
    
    // Extract batch from buffer
    
    auto n_samples                     = m_batch_id;

    Eigen::VectorXf return_batch       = Eigen::VectorXf::Zero(n_samples);
    Eigen::VectorXf adv_batch          = Eigen::VectorXf::Zero(n_samples+1);
    // note last value will never be used as assumes ends in a terminal state which is defines as zero value.
    // For early stopping add in a feed forward final state in NN.
    
    Eigen::VectorXf reward_batch       = m_reward_cache(Eigen::seqN(0, n_samples));
    Eigen::VectorXi done_batch         = m_done_cache(Eigen::seqN(0, n_samples));
    Eigen::VectorXf val_hat_batch      = m_value_hat_cache(Eigen::seqN(0, n_samples+1));
//    Eigen::MatrixXf ob_batch
//    Eigen::MaXf ob_batch      = m_ob_cache(Eigen::seqN(0, n_samples));
//    SHOW2("ob cache", ob_batch);
    
    val_hat_batch[n_samples] = 0;      // note this value will never be used as terminal state
    SHOW2("n samples", n_samples)
    
    
//    Eigen::MatrixXf model_hat_batch       =
    
    // Compute GAE Advantages and Return
    for (std::int32_t i=n_samples-1; i>-1; --i )
    {
        float delta;
        
        // Early stopping on a non-Terimal state
        if(done_batch[i]==2){
            Eigen::VectorXf next_ob = m_next_ob_cache.row(i);
            ForwardPropagate(next_ob);
            float death_value = m_value_hat;
            delta = reward_batch[i] + gamma * death_value - val_hat_batch[i];
        
        // True Terimal state
        } else if(done_batch[i]==1){
            delta = reward_batch[i] - val_hat_batch[i];
            
            
        // Non-Terminal state
        } else {
            delta = reward_batch[i] + gamma * val_hat_batch[i+1] - val_hat_batch[i];
        }
        
        adv_batch[i] = delta + gamma * lambda * adv_batch[i + 1] * (not done_batch[i]);
    }
    
    adv_batch = adv_batch(Eigen::seqN(0, n_samples));
    val_hat_batch = val_hat_batch(Eigen::seqN(0, n_samples));
    return_batch = val_hat_batch + adv_batch;
    
    //ToDo shorten returns and adv by 1
    
    
    // Normalise Advantages
    float adv_mean = adv_batch.mean();
    float adv_std = 0;
    for (std::int32_t i=0; i < n_samples; ++i){
        adv_std += pow(adv_batch[i] - adv_mean, 2.0f);
    }
    adv_std = std::sqrt( adv_std / ( (float(n_samples)-1) + float(1e-8)) );  //unbiased std estimate and avoid NaNs with 1e-8
    
    // set Mean=0, and Standard Deviation=1
    for (std::int32_t i=0; i < m_batch_id; ++i){
        adv_batch[i] = (adv_batch[i] - adv_mean) / (adv_std + float(1e-8));
    }
    m_return_cache = return_batch;
    m_adv_cache = adv_batch;
    
    // User logging only
    m_avg_eps_reward_cumsum = 0;
    std::uint32_t n_episodes = 0;
    for (std::int32_t i=0; i < m_batch_id; ++i){
        if (done_batch[i] == true) {
            n_episodes +=1;
        };
        m_avg_eps_reward_cumsum += reward_batch[i];
    }
    m_avg_eps_reward_cumsum /= float(n_episodes);
};


bool Agent::UpdatePPO(std::uint16_t n_epochs){
    CalcLogProbRev();

    // Compute GAE and Normalise
    ReturnGAE();
    
    // Compute min_batch sizes
    std::uint16_t n_minibatches = int(ceil(m_batch_id / m_minibatch_size));
    std::uint16_t final_minibatch_size;
    if (n_minibatches==0){
        SHOW("RLError: n_minibatches is 0");
        return true;
    }
    if ((m_batch_id % m_minibatch_size) !=0) {
        final_minibatch_size = m_batch_id % m_minibatch_size;
        n_minibatches +=1;
    } else {
        final_minibatch_size = m_minibatch_size;
    }
    
    for(std::int16_t e=0; e < n_epochs; ++e){
        // Shuffle batch_ids
        auto IDs = Eigen::VectorXi(m_batch_id);
        for (std::uint16_t i=0; i < m_batch_id; ++i){IDs[i] = i;}
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
            int mini_size = int(mini_IDs.size());
            Eigen::MatrixXf mini_ob         = Eigen::MatrixXf(m_in_size, mini_size);
            Eigen::MatrixXf mini_act_old    = Eigen::MatrixXf(m_act_size, mini_size);
            Eigen::MatrixXf mini_breakdown = Eigen::MatrixXf(m_act_size, mini_size);
            Eigen::VectorXf mini_log_old    = Eigen::VectorXf(mini_size);
            Eigen::VectorXf mini_log_new    = Eigen::VectorXf(mini_size);
            Eigen::MatrixXf mini_return     = Eigen::MatrixXf(1, mini_size);
            Eigen::VectorXf mini_advantage  = Eigen::VectorXf(mini_size);
            Eigen::VectorXf mini_ratio      = Eigen::VectorXf(mini_size);
            Eigen::VectorXf mini_surrogate1 = Eigen::VectorXf(mini_size);
            Eigen::VectorXf mini_surrogate2 = Eigen::VectorXf(mini_size);
            Eigen::VectorXf mini_ppo_fix    = Eigen::VectorXf(mini_size);
            Eigen::VectorXi mini_clipped    = Eigen::VectorXi(mini_size);

            for(int i=0; i < mini_size; ++i){
                int id = mini_IDs[i];
                mini_ob.col(i)      = m_ob_cache.row(id);
                mini_act_old.col(i) = m_act_cache.row(id);
                mini_log_old(i)     = m_log_prob_cache(id);
                mini_return(i)      = m_return_cache(id);
                mini_advantage(i)   = m_adv_cache(id);
            }

            pNet.ForwardPropagateMulti(mini_ob);
            vNet.ForwardPropagateMulti(mini_ob);

            /// collection probabilty of old act with current network
            for(int cc=0; cc<mini_size; ++cc){
                for(int ii=0; ii<m_act_size; ++ii){
                  mini_breakdown.col(cc)(ii) = GaussianLikelihood1D(pNet.m_A3b.col(cc)(ii), m_standard_deviation, mini_act_old.col(cc)(ii));
                }
                mini_log_new(cc) =log(mini_breakdown.col(cc).prod());
            }
            mini_ratio = (mini_log_new-mini_log_old).array().exp();
            mini_surrogate1 = mini_ratio.cwiseProduct(mini_advantage);
            for(int cc=0; cc<mini_size; ++cc){
                mini_surrogate2(cc) =  std::clamp(mini_ratio(cc), 1.0f - ip_clip, 1.0f + ip_clip) * mini_advantage(cc);
                if(mini_surrogate2(cc) < mini_surrogate1(cc)){
                    mini_ppo_fix(cc) = 0;
                } else {
                    mini_ppo_fix(cc) = (mini_advantage(cc) / mini_log_old(cc));
                }
            }
            vNet.BackPropagateMulti(mini_return, mini_ob);
            pNet.BackPropagateMulti(mini_act_old, mini_ob, mini_ppo_fix);
        }
    }
    m_batch_id = 0;
    SHOW2("Difference: ", m_sample_var);
    m_sample_var = 0;
    return false;
};

void Agent::LinearAnneal(){
    if (m_standard_deviation > m_min_standard_deviation) {m_standard_deviation -= m_anneal_rate;}
}

void Agent::ZeroAdamMomentums(){
    
    vNet.ZeroAdamMomentums();
    pNet.ZeroAdamMomentums();
}

void Agent::SaveModelData(const char * resourcePath, const char * jsonFile, const int iter){
    
    chdir(resourcePath);
    std::string tmp_name = std::to_string(iter) + '_' + jsonFile;
    
    nlohmann::json jm;
    jm = nlohmann::json::parse(R"([])");
    int a_size = m_act_size;
    int e_size = m_in_size;
        
    Eigen::VectorXf x(a_size+e_size);
    for(uint16_t i=0; i < m_batch_id; i++)
    {
        for(uint16_t ii = 0; ii < e_size; ii++){x[ii] = m_ob_cache.row(i)[ii];}
        for(uint16_t ii = e_size; ii < e_size+a_size; ii++){x[ii] = m_act_cache.row(i)[ii-e_size];}
            jm[0]["x"][i] = x;
            jm[0]["y"][i] = m_next_ob_cache.row(i);
    }
    std::ofstream om(tmp_name);
    om << std::setw(4) << jm << std::endl;
}

void Agent::SampleAction(){
    /// Samples an action depending on the selection mode m_maas_mode and m_running_act (i.e. 1-beta)

    // Assign symmetry pairs
    Eigen::VectorXi pair_ids(8);
    bool            pair_mirror;            /// true = inverted mirror symetry (replace with vector if want to vary individual action features)
    bool            pair_temporal;
    
    if(m_maas_mode == 0){                   ///temporal symetry (vanilla RL when filter=1)
        pair_ids << 0, 1, 2, 3, 4, 5, 6, 7;
        pair_temporal = true;
        pair_mirror =  false;
        
    } else if(m_maas_mode == 1) {           /// temporal gallop from sagittal mirror plane
        pair_ids << 2, 3, 0, 1, 6, 7, 4, 6;
        pair_temporal = true;
        pair_mirror =  false;
        
    } else if(m_maas_mode == 2) {           /// temporal crawl from transverse mirror plane
        pair_ids << 4, 5, 6, 7, 0, 1, 2, 3;
        pair_temporal = true;
        pair_mirror   = false;
        
    } else if(m_maas_mode == 3) {           /// temporal trot  (negative gallop) from reverse sagittal mirror plane
        pair_ids << 2, 3, 0, 1, 6, 7, 4, 6;
        pair_temporal = true;
        pair_mirror   = true;
        
    } else if(m_maas_mode == 4) {           /// gallop from sagittal mirror plane
        pair_ids << 2, 3, 0, 1, 6, 7, 4, 6;
        pair_temporal = false;
        pair_mirror   = false;
        
    } else if(m_maas_mode == 5) {           /// crawl from transverse mirror plane
        pair_ids << 4, 5, 6, 7, 0, 1, 2, 3;
        pair_temporal = false;
        pair_mirror   = false;
        
    } else if(m_maas_mode == 6) {           /// trot  (negative gallop) from reverse sagittal mirror plane
        pair_ids << 2, 3, 0, 1, 6, 7, 4, 6;
        pair_temporal = false;
        pair_mirror   = true;
    } else {
        SHOW2("User Error, invalid m_maas_mode: ", m_maas_mode)
        pair_temporal = false;
        pair_mirror   = false;
    }
    
    float rev = 1.0;
    if(pair_mirror){rev = -1.0;}
    
    if(m_first && pair_temporal){ /// No memory as first sample
        m_first = false;
        for(std::uint16_t i=0; i<m_act_size; ++i){
            std::normal_distribution<double> distribution(m_mu(i), m_standard_deviation);
            m_act(i) = (distribution(m_generator));
        }
        
    } else { /// if temporal memory propotional to m_running_act, else m_running_act is sample from pair distribution at current step
        for(std::uint16_t i=0; i<m_act_size; ++i){
            std::normal_distribution<double> distribution(m_mu(i), m_standard_deviation);
            if(pair_temporal){
                m_act(i) = distribution(m_generator)*(1.0-m_running_act) + rev*m_prev_act(pair_ids(i))*m_running_act;
            } else {
                std::normal_distribution<double> distributionP(m_mu(pair_ids(i)), m_standard_deviation);
                m_act(i) = distribution(m_generator)*(1.0-m_running_act) + rev*distributionP(m_generator)*m_running_act;
            }
        }
    }
    m_prev_act = m_act; // only required if temporal MAAS
    
/// For comparison this is what vanilla RL sampling looks like -----------------------------------------------------------------------------
//    for(std::uint16_t i=0; i<m_act_size; ++i){
//        std::normal_distribution<double> distribution(m_mu(i), m_standard_deviation);
//        m_act(i) = (distribution(m_generator));
//    }
/// -----------------------------------------------------------------------------------
}
