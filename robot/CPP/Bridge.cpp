//
//  bridge.cpp
//  RL
//
//  Created by Kez Smithson Whitehead on 29/03/2022.
//

#include "Agent.hpp"
#include "NN.hpp"
#include "EnvDrifter.hpp"
#include "EnvPiper.hpp"
#include "EnvCartPole.hpp"
#include "EnvCartPoleMoCap.hpp"
#include "EnvHCartPole.hpp"
#include "EnvHHCartPole.hpp"
#include "EnvCartPoleMDP.hpp"
#include "EnvLidar.hpp"
#include "EnvSmallQuadruped.hpp"
#include "EnvLinkNone.hpp"
#include "EnvLinkLim.hpp"
#include "EnvLinkSim.hpp"
#include "EnvLinkUn.hpp"
#include "EnvDragger.hpp"
#include "EnvCheetah.hpp"
#include "EnvTrolley.hpp"
#include <unistd.h>
#include <json.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <filesystem>

// Extern c ====================================================

extern "C"
{

class MyClass {       // The class
  public:             // Access specifier
    int myNum;        // Attribute (int variable)
    std::string myString;  // Attribute (string variable)
};

float gaitScores[3];
float gaitScoresTrain[3];
float joint_range = 2.0;
Agent *agent = nullptr;
CoreEnv *env = nullptr;
CartPoleMoCap *MoCapEnv = nullptr;
//nlohmann::json j = nlohmann::json::parse(R"([])");
//
//nlohmann::json jm = nlohmann::json::parse(R"([])");

int seed = 1;
uint16_t ob_mode = 1;
float lr = 0.0003;
uint16_t h1 = 8;
uint16_t h2 = 8;
uint16_t n_epochs = 10;  //10
uint16_t mini_batch_size = 64;
uint16_t samples_per_iter = 600;
uint32_t step_limit = 200;
std::string env_name = "cartpole";
bool adam = true;
uint32_t total_training_steps = 0;
std::string m_path;
std::string m_sub_file;
std::string m_file;
int m_iter;
int m_sub_iter=-1;
std::string m_port_name;
float m_episode_cum;
float m_batch_cum;
float m_batch_cum_det;
int m_eps;
int m_eps_det;

int steps[1000];  // must be outside function
float socs[1000];  // must be outside function
float dets[1000];  // must be outside function
float vals[441];
float pol[441];
float piTruth[441];


void deleteJson(const char * resourcePath, const char * jsonFile){
    chdir(resourcePath);
    
    char iFile[100] = "info_";
    char mFile[100] = "model_";
    char dFile[100] = "data_";
    char vFile[100] = "value_";
    char pFile[100] = "policy_";
    
    strcat(iFile, jsonFile);
    strcat(mFile, jsonFile);
    strcat(dFile, jsonFile);
    strcat(vFile, jsonFile);
    strcat(pFile, jsonFile);
    
    
    // Load info to see how many iterations to delete
    SHOW(iFile)
    
    std::ifstream i(iFile);
    if(!i.fail()){
        nlohmann::json j;
        i >> j;
        int n_iter = int(j.size())-1;
        
        
        for (int i = 0; i <= n_iter; ++i) {
            std::string tmp_name = std::to_string(i) + '_' + mFile;
            SHOW2("deleteing ", tmp_name)
            std::remove(tmp_name.c_str());
        }
        for (int i = 0; i <= n_iter; ++i) {
            std::string tmp_name = std::to_string(i) + '_' + dFile;
            SHOW2("deleteing ", tmp_name)
            std::remove(tmp_name.c_str());
        }
        for (int i = 0; i <= n_iter; ++i) {
            std::string tmp_name = std::to_string(i) + '_' + vFile;
            SHOW2("deleteing ", tmp_name)
            std::remove(tmp_name.c_str());
        }
        for (int i = 0; i <= n_iter; ++i) {
            std::string tmp_name = std::to_string(i) + '_' + pFile;
            SHOW2("deleteing ", tmp_name)
            std::remove(tmp_name.c_str());
        }
        std::remove(iFile);
    }
};

void deleteJsonRange(const char * resourcePath, const char * jsonFile, int upperLim){
    chdir(resourcePath);
    
    char iFile[100] = "info_";
    char mFile[100] = "model_";
    char dFile[100] = "data_";
    char vFile[100] = "value_";
    char pFile[100] = "policy_";
    
    strcat(iFile, jsonFile);
    strcat(mFile, jsonFile);
    strcat(dFile, jsonFile);
    strcat(vFile, jsonFile);
    strcat(pFile, jsonFile);
    
    
    // Load info to see how many iterations to delete
    SHOW(iFile)
    
    std::ifstream i(iFile);
    if(!i.fail()){
        //replace ifile
        nlohmann::json j;
        i >> j;
        int n_iter = int(j.size())-1;
        
        nlohmann::json jn;
        jn = nlohmann::json::parse(R"([])");
        for (int i = 0; i < upperLim; ++i) {
            jn[i] = j[i];
        }
        std::remove(iFile);
        std::ofstream o(iFile);
        o << std::setw(4) << jn << std::endl;
        
        
        for (int i = upperLim; i <= n_iter; ++i) {
            std::string tmp_name = std::to_string(i) + '_' + mFile;
            SHOW2("deleteing ", tmp_name)
            std::remove(tmp_name.c_str());
        }
        for (int i = upperLim; i <= n_iter; ++i) {
            std::string tmp_name = std::to_string(i) + '_' + dFile;
            SHOW2("deleteing ", tmp_name)
            std::remove(tmp_name.c_str());
        }
        for (int i = upperLim; i <= n_iter; ++i) {
            std::string tmp_name = std::to_string(i) + '_' + vFile;
            SHOW2("deleteing ", tmp_name)
            std::remove(tmp_name.c_str());
        }
        for (int i = upperLim; i <= n_iter; ++i) {
            std::string tmp_name = std::to_string(i) + '_' + pFile;
            SHOW2("deleteing ", tmp_name)
            std::remove(tmp_name.c_str());
        }
    }
};

void EnvSelector(std::string envName,
                 uint16_t seed){
    
    SHOW2("env:", envName)
//    SHOW2("port:", portName_)
    if (envName=="cartpolePD"){
        env = new CartPoleEnv(envName, seed, step_limit, 0);
    } else if (envName=="cartpoleP"){
        env = new CartPoleEnv(envName, seed, step_limit, 1);
    } else if (envName=="cartpolePI"){
        env = new CartPoleEnv(envName, seed, step_limit, 2);
    } else if (envName=="cartpolePID"){
        env = new CartPoleEnv(envName, seed, step_limit, 3);
    } else if (envName=="cartpoleClassicI"){
        env = new CartPoleEnv(envName, seed, step_limit, 4);
    } else if (envName=="cartpoleClassic"){
        env = new CartPoleEnv(envName, seed, step_limit, 5);
    }else if (envName=="dragger"){
        env = new DraggerEnv(envName, seed, step_limit);
    }else if (envName=="cheetah_baseline"){
        env = new CheetahEnv(envName, seed, step_limit);
    }else if (envName=="drifter"){
        env = new DrifterEnv(envName, seed, step_limit);
        
    }else if (envName=="piperPVPV"){
        env = new PiperEnv(envName, seed, step_limit, 3);
    }else if (envName=="piperAll"){
        env = new PiperEnv(envName, seed, step_limit, 5);
    }else if (envName=="trolley"){
        env = new TrolleyEnv(envName, m_port_name.c_str(), seed, step_limit);
    }else if (envName=="cartpoleH"){
        env = new CartPoleEnvH(envName, m_path.c_str(), "primary.json", -1, seed, step_limit);
    }else if (envName=="cartpoleHH"){
        env = new CartPoleEnvHH(envName, m_path.c_str(), "primary.json", "secondary.json", -1, -1, seed, step_limit);
    }else if (envName=="car"){
        env = new LidarEnv(envName, seed, step_limit);
    }else if (envName=="smallQuadruped"){
        env = new SmallQuadrupedEnv(envName, seed, step_limit);
    }else if (envName=="link_none" || envName=="MAPASgallop" || envName=="MAPAScrawl" || envName=="MAPAStrot" || envName=="PASgallop" || envName=="PAScrawl" || envName=="PAStrot"){
        env = new LinkNoneEnv(envName, seed, step_limit);
    }else if (envName=="link_unlimited" || envName=="link_clock"){
        env = new LinkUnEnv(envName, seed, step_limit);
        if(envName=="link_clock"){env->m_clock = true;}
    }else if (envName=="link_lim"){
        env = new LinkLimEnv(envName, seed, step_limit);
    }else if (envName=="link_symmetry"){
        env = new LinkSimEnv(envName, seed, step_limit);
    }else if (envName=="Classic_Primary"){ //For loading real world
        env = new CartPoleEnv(envName, seed, step_limit, 5);
    } else{
        SHOW("Error: No valid env")
    }
}

bool checkEarlyQuit(){
    if (env->m_quit){
        delete agent;
        delete env;
        agent = nullptr;
        env = nullptr;
        return true;
    }
    return false;
}

void deleteAgentandEnv(){
    delete agent;
    delete env;
    delete MoCapEnv;
    agent = nullptr;
    env = nullptr;
    MoCapEnv = nullptr;
}

void swiftCancel(){
    env->m_quit = true;
}


float start_obs[100];


void setFramerate(const int fr_){
    GLFWStuff::framerate = fr_;
}

void setNoise(const float noise_){
    env->m_sensor_noise = noise_;
}


void setSetPoint(const float sp_){
    GLFWStuff::sp = sp_;
}


void PrintMjVersion(const char * a){
    std::cout << "Mujoco Version:\t " << mjVERSION_HEADER << std::endl;
}


int agentGetTotalSteps(){
    return total_training_steps;
}


int loadNetwork(int iter, const int agentOnly){
    
    SHOW("Loading...")
    chdir(m_file.c_str());
    
    SHOW(iter)
    
    char iFile[100] = "info_";
    char mFile[100] = "model_";
    char dFile[100] = "data_";
    char vFile[100] = "value_";
    char pFile[100] = "policy_";
    
    strcat(iFile, m_file.c_str());
    strcat(mFile, m_file.c_str());
    strcat(dFile, m_file.c_str());
    strcat(vFile, m_file.c_str());
    strcat(pFile, m_file.c_str());
    
    std::ifstream i(iFile);
    if(i.fail()){
        iter = -1;
        SHOW("RLByres Warning no iteration found")
    }
    nlohmann::json j;
    i >> j;
    if(iter==-1){iter = int(j.size())-1;}
    SHOW2("Selected iter", iter)

    SHOW2("pFile", pFile)
    SHOW2("resource path:\t", m_path)
    SHOW2("jsob_file:\t", m_file)
    SHOW2("jsob_iter:\t", iter)
    SHOW2("j[iter][step_limit]", j[iter]["step_limit"])
    
    step_limit = j[iter]["step_limit"];
    total_training_steps = j[iter]["total_training_steps"];
    
    
    delete agent;
    if(agentOnly==0){
        delete env;
        EnvSelector(j[iter]["env"], seed);
    }

    agent = new Agent(j[iter]["in_size"] ,
                      j[iter]["act_size"],
                      j[iter]["h1_size"],  // h1 size
                      j[iter]["h2_size"],  // h2 size
                      lr,
                      seed,
                      j[iter]["desired_batch_size"],
                      j[iter]["minibatch_size"],
                      j[iter]["step_limit"],
                      false,
                      adam);
    agent->pNet.LoadNetwork(m_path.c_str(), pFile, iter);
    agent->vNet.LoadNetwork(m_path.c_str(), vFile, iter);
//    agent->mNet.LoadNetwork(m_path.c_str(), mFile, iter);
    
    if(agentOnly==0){
        if (env_name=="MAPASgallop"){
            agent->m_maas_mode = 1;
        }else if (env_name=="MAPAScrawl"){
            agent->m_maas_mode = 2;
        } else if (env_name=="MAPAStrot"){
            agent->m_maas_mode = 3;
        } else if (env_name=="PASgallop"){
            agent->m_maas_mode = 4;
        }else if (env_name=="PAScrawl"){
            agent->m_maas_mode = 5;
        } else if (env_name=="PAStrot"){
            agent->m_maas_mode = 6;
        }
    }
    
    return int(j.size());
}


const char * ENV_LIST = "cartpoleP,cartpolePI,cartpolePID,cartpolePD,cartpoleClassic,cartpoleClassicI,cartpoleH,cartpoleHH,drifter,dragger,cheetah_baseline,piperPVPV,piperAll,car,smallQuadruped,link_none,link_lim,link_unlimited,link_clock,link_symmetry,MAPASgallop,MAPAScrawl,MAPAStrot,PASgallop,PAScrawl,PAStrot";  //ToDO automate list generation
const char * listEnvs(){
    return ENV_LIST;
};

char char_array_json[1000];
const char * listJsonFiles(const char * resourcePath){
    
    chdir(resourcePath);
    std::string rp = resourcePath;

    // set to string array
    std::string jString= "";
    for (const auto & entry : std::filesystem::directory_iterator(resourcePath)) {
        std::string t = entry.path();
        std::string c = t.erase(0, rp.length()+1);
        bool side_file = false;
        for (int s = 0; s < c.length(); ++s) {
            if(c[s]=='_')side_file=true;
        }
        if (c[0]=='i' and c[1]=='n' and c[2]=='f' and c[3]=='o' and c[4]=='_'){
    
            jString += c.substr(5, c.length());
            jString += ",";
        }
    }
    // declaring character array
    strcpy(char_array_json, jString.c_str());
    return char_array_json;
}


// Set Parameters ----------------------
//void setaPath(const char * path_){
//    m_path = path_;
//}
//
//void setFile(const char * file_name_){
//    m_file = file_name_;
//}
//
//void setSubFile(const char * sub_file_name_){
//    m_sub_file = sub_file_name_;
//}
//
void setPort(const char * port_name_){
    m_port_name = port_name_;
    SHOW2("Port is\t", m_port_name)
}

void setIter(int iter_){
    m_iter = iter_;
}


void setEnv(const char * name){
    env_name = name;
    SHOW2("Set env\t\t", env_name)
}

void setWatching(int watch_){
    if(env != nullptr){
        env->m_watching = watch_;
        SHOW2("watching set to", env->m_watching)
    }
    if(MoCapEnv != nullptr){
        MoCapEnv->m_watching = watch_;
    }
}

void setSeed (int seed_){
    seed = seed_;
}

void setObMode (int mode_){
    ob_mode = mode_;
}

void setH1 (int h1_){
    h1 = h1_;
}

void setH2 (int h2_){
    h2 = h2_;
}

void setLr (float lr_){
    lr = lr_;
}

void setEpochs (int n_epochs_){
    n_epochs = n_epochs_;
}

void setMinibatch (int mini_batch_size_){
    mini_batch_size = mini_batch_size_;
    SHOW(mini_batch_size)
}

void setIterSamples (int samples_per_iter_){
    SHOW2("samples_per_iter", samples_per_iter_)
    samples_per_iter = samples_per_iter_;
}

void setStepLimit(int step_limit_){
    step_limit = step_limit_;
}

void setAdam(int adam_){
    adam = bool(adam_);
}

// Get Parameters ---------------------
std::string comEnv;
const char * getEnv(const char * resourcePath, const char * jsonFile){
    
    char iFile[100] = "info_";
    strcat(iFile, jsonFile);
    chdir(resourcePath);
    std::ifstream i(iFile);
    nlohmann::json j;
    i >> j;
    int ID = int(j.size())-1;
    
    comEnv =  j[ID]["env"];
    
    return comEnv.c_str();
}

int comParams[8];
const int * getIntData(const char * resourcePath, const char * jsonFile){
    char iFile[100] = "info_";
    strcat(iFile, jsonFile);
    chdir(resourcePath);
    std::ifstream i(iFile);
    nlohmann::json j;
    i >> j;
    int ID = int(j.size())-1;
    
    comParams[0] =  j[ID]["h1_size"];
    comParams[1] =  j[ID]["h2_size"];
    comParams[2] =  j[ID]["epochs"];
    comParams[3] =  j[ID]["minibatch_size"];
    comParams[4] =  j[ID]["desired_batch_size"];
    comParams[5] =  j[ID]["step_limit"];
    comParams[6] =  j[ID]["adam"];
    comParams[7] =  j[ID]["seed"];
    
    return comParams;
}

float getLr(const char * resourcePath, const char * jsonFile){
    char iFile[100] = "info_";
    strcat(iFile, jsonFile);
    chdir(resourcePath);
    std::ifstream i(iFile);
    nlohmann::json j;
    i >> j;
    int ID = int(j.size())-1;
    return j[ID]["lr"];
}


char char_array_ports[1000];
const char * listPorts(const char * resourcePath){
    
    chdir(resourcePath);
    std::string rp = resourcePath;
    
    // change to string array
    std::string connectedPorts= "";
    for (const auto & entry : std::filesystem::directory_iterator("/dev")) {
        std::string t = entry.path();
        if (t[5] == 'c' and t[8] == 'u'){
            connectedPorts += t;
            connectedPorts += ",";
        }
    }
    // declaring character array
    strcpy(char_array_ports, connectedPorts.c_str());
    return char_array_ports;
}

int modelStart(const char * resourcePath, const char * jsonFile, int iter, const char * portName_){
//    loadNetwork(resourcePath, jsonFile, iter, 1);  //agent only
    return agent->m_in_size;
}


float observation_out[10];
float* modelStep(float* observation_in, float* action_in){
    Eigen::VectorXf observationIn = Eigen::VectorXf(agent->m_in_size);
    for (int i = 0; i < agent->m_in_size; ++i) {
        observationIn[i] = observation_in[i];
    }
    
    Eigen::VectorXf actionIn = Eigen::VectorXf(agent->m_act_size);
    for (int i = 0; i < agent->m_act_size; ++i) {
        actionIn[i] = action_in[i];
    }
    return observation_out;
}


float* manualStart(const char * resourcePath, const char * portName_){
    m_path = resourcePath;
    chdir(m_path.c_str());
    delete agent;
    delete env;
    Eigen::VectorXf pid_act = Eigen::VectorXf(1);
//    EnvSelector("cartpole", seed, portName_);
    GLFWStuff::camView = 1;  // track
    env->EpisodeReset();
    for (int i = 0; i < env->GetObSize(); ++i) {
        start_obs[i] = env->m_ob[i];
    }
    return start_obs;
}

int checkUserQuit(){
    return env->m_quit;
}





float pid_obs[10];
float* manualStep(float pid_act0, int iter){
    chdir(m_path.c_str());
    std::ifstream i(m_file);
    nlohmann::json jm;
    i >> jm;
    
    Eigen::VectorXf pid_act = Eigen::VectorXf(1);
    pid_act[0] = pid_act0;
    jm[iter]["before"] = env->m_ob;
    env->MarkovStep(pid_act);                      // updates (env->m_reward, env->m_done env->ob)
    auto obs_ = env->NextOb();                        // set next ob (i.e env->m_ob) to ob (i.e. agent->m_ob)
    jm[iter]["act"] = pid_act;
    jm[iter]["after"] = env->m_ob;
    std::ofstream o("model_data.json");
    o << std::setw(4) << jm << std::endl;
    
    for (int i = 0; i < env->GetObSize(); ++i) {
        pid_obs[i] = obs_[i];
    }
    return pid_obs;
}


void manualTerminate(){
    glfwTerminate();
}


void rlReset(){
    SHOW("RL reset")
    // Reset environment
    total_training_steps = 0;
    chdir(m_path.c_str());
    delete agent;
    delete env;
    m_iter=0;
    EnvSelector(env_name, seed);
    
    agent = new Agent(env->GetObSize(),
                      env->GetActSize(),
                      h1,
                      h2,
                      lr,
                      seed,
                      samples_per_iter,
                      mini_batch_size,
                      env->StepLimit,
                      false,
                      adam);

    if (env_name=="MAPASgallop"){
        agent->m_maas_mode = 1;
    }else if (env_name=="MAPAScrawl"){
        agent->m_maas_mode = 2;
    } else if (env_name=="MAPAStrot"){
        agent->m_maas_mode = 3;
    } else if (env_name=="PASgallop"){
        agent->m_maas_mode = 4;
    }else if (env_name=="PAScrawl"){
        agent->m_maas_mode = 5;
    } else if (env_name=="PAStrot"){
        agent->m_maas_mode = 6;
    }

    if(seed==-1 && env->GetObSize()==1){
       agent->pNet.LoadNetwork(m_path.c_str(), "seed1_input1.json", 0);
       agent->vNet.LoadNetwork(m_path.c_str(), "seed1_input1.json", 0);
   } else if(seed==-2 && env->GetObSize()==1){
       agent->pNet.LoadNetwork(m_path.c_str(), "seed2_input1.json", 0);
       agent->vNet.LoadNetwork(m_path.c_str(), "seed2_input1.json", 0);
   } else if (seed==-3 && env->GetObSize()==1){
       agent->pNet.LoadNetwork(m_path.c_str(), "seed3_input1.json", 0);
       agent->vNet.LoadNetwork(m_path.c_str(), "seed3_input1.json", 0);
   } else if (seed==-4 && env->GetObSize()==1){
       agent->pNet.LoadNetwork(m_path.c_str(), "seed4_input1.json", 0);
       agent->vNet.LoadNetwork(m_path.c_str(), "seed4_input1.json", 0);
   } else if (seed==-5 && env->GetObSize()==1){
       agent->pNet.LoadNetwork(m_path.c_str(), "seed5_input1.json", 0);
       agent->vNet.LoadNetwork(m_path.c_str(), "seed5_input1.json", 0);
       
   }else if(seed==-1 && env->GetObSize()==2){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed1_input2.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed1_input2.json", 0);
    } else if(seed==-2 && env->GetObSize()==2){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed2_input2.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed2_input2.json", 0);
    } else if (seed==-3 && env->GetObSize()==2){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed3_input2.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed3_input2.json", 0);
    } else if (seed==-4 && env->GetObSize()==2){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed4_input2.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed4_input2.json", 0);
    } else if (seed==-5 && env->GetObSize()==2){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed5_input2.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed5_input2.json", 0);
        
    }else if(seed==-1 && env->GetObSize()==3){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed1_input3.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed1_input3.json", 0);
    } else if(seed==-2 && env->GetObSize()==3){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed2_input3.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed2_input3.json", 0);
    } else if (seed==-3 && env->GetObSize()==3){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed3_input3.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed3_input3.json", 0);
    } else if (seed==-4 && env->GetObSize()==3){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed4_input3.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed4_input3.json", 0);
    } else if (seed==-5 && env->GetObSize()==3){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed5_input3.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed5_input3.json", 0);
        
    }else if(seed==-1 && env->GetObSize()==4){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed1_input4.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed1_input4.json", 0);
    } else if(seed==-2 && env->GetObSize()==4){
        
        agent->pNet.LoadNetwork(m_path.c_str(), "seed2_input4.json", 0);
        
        agent->vNet.LoadNetwork(m_path.c_str(), "seed2_input4.json", 0);
        
    } else if (seed==-3 && env->GetObSize()==4){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed3_input4.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed3_input4.json", 0);
    } else if (seed==-4 && env->GetObSize()==4){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed4_input4.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed4_input4.json", 0);
    } else if (seed==-5 && env->GetObSize()==4){
        agent->pNet.LoadNetwork(m_path.c_str(), "seed5_input4.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "seed5_input4.json", 0);
    } else if (seed==0 && env->GetObSize()==4){
        
        agent->pNet.LoadNetwork(m_path.c_str(), "policy2_16.json", 0);
        agent->vNet.LoadNetwork(m_path.c_str(), "value2_16.json", 0);
    }
}




void manReset(){
    SHOW("Manual reset")
    delete env;
    EnvSelector(env_name, seed);
}

void batchReset(){
    m_batch_cum = 0;
    m_batch_cum_det = 0;
    m_eps = 0;
    m_eps_det = 0;
    
    gaitScoresTrain[0]=0;
    gaitScoresTrain[1]=0;
    gaitScoresTrain[2]=0;
}

void rlEpisodeReset(int mode){
    SHOW("Env Reset")
    m_episode_cum = 0;
//    agent->m_prev_act.setZero();
//    agent->m_first = true; // just for running average tests uncomment for mapas
    env->EpisodeReset(mode);

    gaitScores[0] = 0;
    gaitScores[1] = 0;
    gaitScores[2] = 0;
    
}

void rlMocapEpisodeReset(){
    SHOW("Mocap Env Reset")
    MoCapEnv->EpisodeReset();
    agent->m_ob = MoCapEnv->m_ob;
}



float* getGaitScores(){
    return gaitScores;
}


int rlStep(int deterministic){
    agent->ForwardPropagate(env->m_ob);
    env->m_deterministic = deterministic;
    if(deterministic) {
        env->MarkovStep(agent->m_mu);
        m_episode_cum += env->m_reward;  // logging
        
        if(env->GetActSize()==8){
            gaitScores[0] += ( 4.0*joint_range -
            abs(env->m_ob[2]-env->m_ob[4]) -
            abs(env->m_ob[3]-env->m_ob[5]) -
            abs(env->m_ob[6]-env->m_ob[8]) -
            abs(env->m_ob[7]-env->m_ob[9]));
            
            gaitScores[1] += (4.0*joint_range -
            abs(env->m_ob[2]-env->m_ob[6]) -
            abs(env->m_ob[3]-env->m_ob[7]) -
            abs(env->m_ob[4]-env->m_ob[8]) -
            abs(env->m_ob[5]-env->m_ob[9]));
            
            gaitScores[2] += (
            abs(env->m_ob[2]-env->m_ob[4]) +
            abs(env->m_ob[3]-env->m_ob[5]) +
            abs(env->m_ob[6]-env->m_ob[8]) +
            abs(env->m_ob[7]-env->m_ob[9]));
        }
        if(env->m_done>0){
            m_eps_det += 1;
            m_batch_cum_det += m_episode_cum;
            SHOW2("gallop\t", gaitScores[0]/float(env->step));
            SHOW2("crawl\t", gaitScores[1]/float(env->step));
            SHOW2("trot\t", gaitScores[2]/float(env->step));
            
            SHOW2("gallopT\t", gaitScoresTrain[0]/float(agent->m_batch_id));
            SHOW2("crawlT\t", gaitScoresTrain[1]/float(agent->m_batch_id));
            SHOW2("trotT\t", gaitScoresTrain[2]/float(agent->m_batch_id));
        }
    } else {
        env->MarkovStep(agent->m_act);
        agent->CacheSample(env->m_reward, env->m_done, env->m_ob);
        total_training_steps += 1;
        m_episode_cum += env->m_reward;  // logging
        if(env->GetActSize()==8){
            gaitScoresTrain[0] += ( 4.0*joint_range -
            abs(env->m_ob[2]-env->m_ob[4]) -
            abs(env->m_ob[3]-env->m_ob[5]) -
            abs(env->m_ob[6]-env->m_ob[8]) -
            abs(env->m_ob[7]-env->m_ob[9]));
            
            gaitScoresTrain[1] += (4.0*joint_range -
            abs(env->m_ob[2]-env->m_ob[6]) -
            abs(env->m_ob[3]-env->m_ob[7]) -
            abs(env->m_ob[4]-env->m_ob[8]) -
            abs(env->m_ob[5]-env->m_ob[9]));
            
            gaitScoresTrain[2] += (
            abs(env->m_ob[2]-env->m_ob[4]) +
            abs(env->m_ob[3]-env->m_ob[5]) +
            abs(env->m_ob[6]-env->m_ob[8]) +
            abs(env->m_ob[7]-env->m_ob[9]));
        }
        if(env->m_done>0){
            m_eps += 1;
            m_batch_cum += m_episode_cum;
        }
    }
    return env->m_done;
}


void rlMocapReset(const char * env_mode){
    SHOW("Mocap reset")
    delete MoCapEnv;
    MoCapEnv = new CartPoleMoCap("cartpoleMocap", seed, step_limit, env_mode);
}


float preditions[100];
float* rlPredict(float* ob_in){
    
    // convert ob to eigen vector
    Eigen::VectorXf obIn = Eigen::VectorXf(MoCapEnv->GetObSize());
    for (int i = 0; i < MoCapEnv->GetObSize(); ++i) {
        obIn[i] = ob_in[i];
    }
    // update action from policy and then next state prediction form model
    obIn[0] -= GLFWStuff::sp;
    
    agent->ForwardPropagate(obIn);
//    agent->mNet.m_in[agent->mNet.m_in_size-1]=agent->m_mu[0];
    agent->mNet.ForwardPropagate();
    SHOW2("\nModel In",agent->mNet.m_in);
    SHOW2("Model Out",agent->mNet.m_out);
    SHOW2("Act",agent->m_mu[0]);
    SHOW("\n")
//
    if(MoCapEnv->m_env_mode == "cartpolePI"){
        MoCapEnv->MoCapStep(agent->mNet.m_out[0], 0);
    } else {
        MoCapEnv->MoCapStep(agent->mNet.m_out[0], agent->mNet.m_out[3]);
    }

    
    MoCapEnv->m_ob = agent->mNet.m_out;
    MoCapEnv->m_act = agent->m_mu;

    
    // convert eigen ob to vector
    for (int i = 0; i < MoCapEnv->GetObSize(); ++i) {
        preditions[i] = agent->mNet.m_out[i];
    }
    return preditions;
}


int manStep(float* act_in){

    // convert act to eigen vector
    Eigen::VectorXf actIn = Eigen::VectorXf(env->GetActSize());
    for (int i = 0; i < env->GetActSize(); ++i) {
        actIn[i] = act_in[i];
    }
    
    // step
    env->MarkovStep(actIn);
//    SHOW(env->m_reward);
    
    // return done
    return env->m_done;
}
    

float obs[100];
float* getAgentOb(){
    for (int i = 0; i < agent->m_in_size; ++i) {
        obs[i] = agent->m_ob[i];
    }
    return obs;
}

float* getNewValueProbe(int save_,int o1, int o2){

    float inc = 0.1;
    int id_ = 0;
    std::ofstream myFile("probeV.csv");
    
    for (int x_ = 0; x_ < agent->m_in_size; ++x_) {
        agent->vNet.m_in[x_] = 0; // condition on zero for other dimentions
    }
    
    if(agent->m_in_size > 1){
        for (int x_ = -10; x_ < 11; ++x_) {
            agent->vNet.m_in[o1] = float(x_) * inc;
            for (int y_ = -10; y_ < 11; ++y_) {
                if(agent->m_in_size > 1){agent->vNet.m_in[o2] = float(y_) * inc;}
                agent->vNet.ForwardPropagate();
                vals[id_] = agent->vNet.m_out[0];
                
                myFile << agent->vNet.m_in[o1];
                myFile << '\t';
                if(agent->m_in_size > 1){myFile << agent->vNet.m_in[o2];myFile << '\t';}
                myFile <<  vals[id_];
                myFile << '\n';
                id_+=1;
            }
        }
    }
    return vals;
}

float* getNewPolicyProbe(int save_,int o1, int o2){
    float inc = 0.1;
    int id_ = 0;
    std::ofstream myFile("probeP.csv");
    
    for (int x_ = 0; x_ < agent->m_in_size; ++x_) {
        agent->pNet.m_in[x_] = 0; // condition on zero for other dimentions
    }

    for (int x_ = -10; x_ < 11; ++x_) {
        agent->pNet.m_in[o1] = float(x_) * inc;
        for (int y_ = -10; y_ < 11; ++y_) {
            if(agent->m_in_size > 1){agent->pNet.m_in[o2] = float(y_) * inc;}

            agent->pNet.ForwardPropagate();
            pol[id_] = agent->pNet.m_out[0];
            
            myFile << agent->pNet.m_in[o1];
            myFile << '\t';
            if(agent->m_in_size > 1){myFile << agent->pNet.m_in[o2];myFile << '\t';}
            myFile <<  pol[id_];
            myFile << '\n';
            
            
            id_+=1;
            
        }
    }
    myFile.close();
    return pol;
}


float act[10];
float* getAct(){
    for (int i = 0; i < agent->m_act_size; ++i) {
        act[i] = agent->m_act[i];
    }
    return act;
}

// Next obs
float* getEnvOb(){
    for (int i = 0; i < env->GetObSize(); ++i) {
        obs[i] = env->m_ob[i];
    }
    return obs;
}

int getObDim(){
    return env->GetObSize();
}

int getAgentObDim(){
    return agent->m_in_size;
}

float getReward(){
    return env->m_reward;
}

float getReturn(){
    return m_episode_cum;
}

int getDone(){
    return env->m_done;
}

int getQuit(){
    return int(env->m_quit);
}

int getBatchFull(){
    if(agent->m_batch_id < agent->m_desired_batch_size){
        return 0;
    } else {
        return 1;
    }
}

void rlSave(int iter){
    
    // Save
    char mFile[100] = "model_";
    char dFile[100] = "data_";
    char vFile[100] = "value_";
    char pFile[100] = "policy_";
    strcat(mFile, m_file.c_str());
    strcat(dFile, m_file.c_str());
    strcat(vFile, m_file.c_str());
    strcat(pFile, m_file.c_str());
    
    
//    saveNetwork(iter);
    agent->pNet.SaveNetwork(m_path.c_str(), pFile, iter);
    agent->vNet.SaveNetwork(m_path.c_str(), vFile, iter);
    agent->mNet.SaveNetwork(m_path.c_str(), mFile, iter);
    agent->SaveModelData(m_path.c_str(), dFile, iter);
    
    chdir(m_path.c_str());
    char iFile[100] = "info_";
    strcat(iFile, m_file.c_str());
    
    nlohmann::json j;
    if(iter==0){
        j = nlohmann::json::parse(R"([])");
    } else {
        std::ifstream i(iFile);
        i >> j;
    }
    j[iter]["epochs"] = n_epochs;
    j[iter]["adam"] = agent->m_adam;
    j[iter]["step_limit"] = env->StepLimit;
    j[iter]["lr"] = agent->m_lr;
    j[iter]["seed"] = agent->m_seed;
    j[iter]["desired_batch_size"] = agent->m_desired_batch_size;
    j[iter]["minibatch_size"] = agent->m_minibatch_size;
    j[iter]["deterministic_return"] = m_batch_cum_det/m_eps_det;
    j[iter]["socastic_return"] = m_batch_cum/m_eps;
    j[iter]["ID"] = iter;
    j[iter]["env"] = env->Name;
    j[iter]["total_training_steps"] = total_training_steps;
    j[iter]["in_size"] = agent->m_in_size;
    j[iter]["act_size"] = agent->m_act_size;
    j[iter]["h1_size"] = agent->m_h1_size;
    j[iter]["h2_size"] = agent->m_h2_size;
    j[iter]["actFilter"] = agent->m_running_act;
    
    if(env->GetActSize()==8){
        j[iter]["gait_train"] = gaitScoresTrain;
        j[iter]["gait_test"] = gaitScores;
    }
    
    getNewValueProbe(0,0,1); // updates vals
    getNewPolicyProbe(0,0,1);
//    j[iter]["value_probe"] = vals;
//    j[iter]["policy_probe"] = pol;
    
    std::ofstream o(iFile);
    o << std::setw(4) << j << std::endl;
    
    // delete file to free space
    std::string tmp_name = std::to_string(iter) + '_' + dFile;
    std::remove(tmp_name.c_str());
}

void rlModel(int iter, int epochs){
    char dFile[100] = "data_";
    strcat(dFile, m_file.c_str());
    agent->mNet.LoadDataset(m_path.c_str(), dFile, iter, 0.8);
    agent->mNet.Test();
    agent->mNet.Train(epochs);
    agent->mNet.Test();
    // delete file to free space
    std::string tmp_name = std::to_string(iter) + '_' + dFile;
    std::remove(tmp_name.c_str());
    
}

void ZeroAdamMomentums(){
    agent->ZeroAdamMomentums();
}

void UpdatePPO(){
    agent->UpdatePPO(n_epochs);
}

void LinearAnneal(){
    agent->LinearAnneal();
}


int* getSteps(const char * jsonFile){
    chdir(m_path.c_str());
    char iFile[100] = "info_";
    strcat(iFile, jsonFile);
    SHOW2("steps file ", iFile)
    std::ifstream i(iFile);
    nlohmann::json j;
    i >> j;

    for (int i = 0; i < j.size(); ++i) {
        steps[i] = j[i]["total_training_steps"];
    }
    
    return steps;
}

void setPathAndFile(const char * resourcePath,
                    const char * jsonFile,
                    const char * jsonSubFile,
                    int subIter,
                    const char * portName){
    // do before loadNetwork and before rlReset
    m_path = resourcePath;
    SHOW(resourcePath)
    
    
    m_file = jsonFile;
    m_sub_file = jsonSubFile;
    m_sub_iter = subIter;
    SHOW2("path\t\t", m_path)
    SHOW2("file\t\t", m_file)
    SHOW2("sub file\t", m_sub_file)
    SHOW2("sub iter\t", m_sub_iter)
}

int iterCount(const char * resourcePath, const char * jsonFile){
    char iFile[100] = "info_";
    strcat(iFile, jsonFile);
    chdir(resourcePath);
    std::ifstream i(iFile);
    nlohmann::json j;
    i >> j;
    return int(j.size());
}

float* getStocastic(const char * jsonFile){

    chdir(m_path.c_str());
    char iFile[100] = "info_";
    strcat(iFile, jsonFile);
    std::ifstream i(iFile);
    nlohmann::json j;  // todo can delete as global
    i >> j;
    for (int i = 0; i < j.size(); ++i) {socs[i] = j[i]["socastic_return"];}  //mneed to save!!
    return socs;
}

//float* getValueProbe(const char * jsonFile, int iter){
//
//    chdir(m_path.c_str());
//    char iFile[100] = "info_";
//    strcat(iFile, jsonFile);
//    std::ifstream i(iFile);
//    nlohmann::json j;  // todo can delete as global
//    i >> j;
//    if(iter==-1){iter = int(j.size())-1;}
//    for (int i = 0; i < 441; ++i) {vals[i] = j[iter]["value_probe"][i];}  //mneed to save!!
//    return vals;
//}
//
//float* getPolicyProbe(const char * jsonFile, int iter){
//
//    chdir(m_path.c_str());
//    char iFile[100] = "info_";
//    strcat(iFile, jsonFile);
//    std::ifstream i(iFile);
//    nlohmann::json j;  // todo can delete as global
//    i >> j;
//    if(iter==-1){iter = int(j.size())-1;}
//    for (int i = 0; i < 441; ++i) {pol[i] = j[iter]["policy_probe"][i];}  //mneed to save!!
//    return pol;
//}

float* getDeterministic(const char * jsonFile){
    chdir(m_path.c_str());
    char iFile[100] = "info_";
    strcat(iFile, jsonFile);
    std::ifstream i(iFile);
    nlohmann::json j;
    i >> j;
    for (int i = 0; i < j.size(); ++i) {dets[i] = j[i]["deterministic_return"];}  //mneed to save!!
    return dets;
}

float getSetPoint(){
    return GLFWStuff::sp;
}

float getSetPoint2(){
    return GLFWStuff::sp2;
}

float getTruePoint(){
    return env->m_true;
}

void setActFilter(float filter_){
    agent->m_running_act = filter_;
    SHOW2("action filter set to ", agent->m_running_act)
}

void setDriveMode(int d){
    if(d){
        GLFWStuff::driveMode = true;
    } else {
        GLFWStuff::driveMode = false;
    }
}

void StepLimited(const int limit_step){
    if(limit_step){
        env->StepLimited = true;
    }else {
        env->StepLimited = false;
    }
    SHOW2("limit_step", limit_step)
}


void ExhaustiveSearch(float minP_, float maxP_, float minI_, float maxI_, int n, int mode_){
    
    SHOW2("min P\t", minP_)
    SHOW2("max P\t", maxP_)
    SHOW2("min I\t", minI_)
    SHOW2("max I\t", maxI_)
    float deltaP = (maxP_ - minP_)/float(n);
    float deltaI = (maxI_ - minI_)/float(n);
    env_name = "cartpolePI";
    manReset();
    float t = env->getTimeset();
    Eigen::MatrixXf undiscountedRewards = Eigen::MatrixXf(n+1, n+1);
    int countP = 0;
    for (float p = minP_; p <= maxP_; p += deltaP) {
        int countI = 0;
        for (float v = minI_; v <= maxI_; v += deltaI) {
            
            rlEpisodeReset(2);
            float prev_err = 0;
            float prev_inter = 0;
            float score = 0;
            
            while(!env->m_done){
                Eigen::VectorXf pi_act = Eigen::VectorXf(1);
                float err = env->m_ob[0];
                float prop = p * err;
                float inter = v * (err + prev_err) * t * 0.5 + prev_inter;
                float dif = v * (err - prev_err) / t;
                
                if (mode_==0){
                    pi_act[0] = prop + dif;
                    SHOW("PD")
                } else {
                    pi_act[0] = prop + inter;
                    SHOW("PI")
                }
                
                env->MarkovStep(pi_act);

                prev_err = err;
                prev_inter = inter;
                score += env->m_reward;
            }
            undiscountedRewards(countP,countI) = score;
            countI += 1;
            SHOW(env->m_done)
        }
        countP += 1;
    }
    
    Eigen::Index maxRow, maxCol;
    float max = undiscountedRewards.maxCoeff(&maxRow, &maxCol);
    float best_p = float(maxRow)*deltaP + minP_;
    float best_i = float(maxCol)*deltaI + minI_;
    std::cout << "Max: " << max <<  ", at: " << maxRow << "," << maxCol << std::endl;
    std::cout << "Best P: " << best_p <<  ", best I: " << best_i << std::endl;
    swiftCancel();
    deleteAgentandEnv();
    
    // Save to json
    chdir(m_path.c_str());
    char fileName[100] = "exhaust.json";
    nlohmann::json j;
    j = nlohmann::json::parse(R"([])");
    for(uint16_t i=0; i < undiscountedRewards.rows(); i++){j[i] = undiscountedRewards.row(i);}
    std::ofstream o(fileName);
    o << std::setw(4) << j << std::endl;
}
    
float exhastiveData[10201];
float* getExhastiveData(){
//
    char fileName[100] = "exhaust.json";
    std::ifstream i(fileName);
    nlohmann::json j;
    i >> j;
    SHOW2("p dim",j.size());
    SHOW2("i dim",j[0].size());
    for (int i = 0; i < j.size(); ++i) {
        for (int ii = 0; ii < j.size(); ++ii) {
            exhastiveData[i*j.size()+ii] = j[i][ii];
        }
    }
    return exhastiveData;
}

int getExhaustiveSize(){
    char fileName[100] = "exhaust.json";
    std::ifstream i(fileName);
    nlohmann::json j;
    i >> j;
//    return int(j.size()*j[0].size());
    return int(j.size());
}

float* PIProbe(float kP, float kI){
    float inc = 0.1;
    int id_ = 0;
    std::ofstream myFile("PIprobe.csv");
    std::ofstream myFile2("PIclamped.csv");

    for (int x_ = -10; x_ < 11; ++x_) {
        float ob1 = float(x_) * inc;
        for (int y_ = -10; y_ < 11; ++y_) {
            float ob2 = float(y_) * inc;
            piTruth[id_] = kP*ob1 + kI*ob2;
            float clamped = piTruth[id_];
            
            if(clamped>1.0){
                clamped = 1.0;
            } else if (clamped<-1.0){
                clamped = -1.0;
            }
            
            myFile << ob1;
            myFile << '\t';
            myFile << ob2;
            myFile << '\t';
            myFile <<  piTruth[id_];
            myFile << '\n';
            
            myFile2 << ob1;
            myFile2 << '\t';
            myFile2 << ob2;
            myFile2 << '\t';
            myFile2 <<  clamped;
            myFile2 << '\n';
            
            
            
            
            id_+=1;
        }
    }
    return piTruth;
}

void BasicNN(const char * resourcePath, const char * portName){
    chdir(resourcePath);
    std::string rp = resourcePath;
    NeuralNetwork tmpNet(2,
                         1,
                         4,
                         4,
                         0.0003,
                         1,
                         64,
                         true,
                         false
                        );
    SHOW2("resourcePath", resourcePath)
    tmpNet.LoadNetwork(rp.c_str(), "basic.json", 0);
    
    SHOW(tmpNet.m_W1);
    tmpNet.m_in(0) = 0.2;
    tmpNet.m_in(1) = 0.3;
    tmpNet.ForwardPropagate();
    SHOW2("m_out", tmpNet.m_out);
    
    struct termios tty;
    int serial_port = open(portName, O_RDWR);
    SHOW2("Port Name:\t", portName);
    SHOW2("Serial Port:\t", serial_port);
    
    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    SHOW("here\n")

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 2;    // 10 = Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    //ToDO change baud rate!!!!
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
    
    
    union {
     uint8_t b[4];
     float f;
    } dataOut;
    
    union {
     uint8_t b[4];
     float f;
    } dataIn;
    
    dataOut.f = 69.123;
    write(serial_port, dataOut.b, 4);
    u_int8_t num_bytes = u_int8_t(read(serial_port, &dataIn.b, 4));
    SHOW(dataIn.f);
    
}

void setKillAngle(float a){
    env->m_kill_angle = a;
}


struct termios tty;
int serial_port;
void DongleBegin(const char * portName){
//    struct termios tty;
    serial_port = open(portName, O_RDWR);
    SHOW2("Port Name:\t", portName);
    SHOW2("Serial Port:\t", serial_port);
    
    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    SHOW("here\n")

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 2;    // 10 = Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    //ToDO change baud rate!!!!
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

void DongleCommand(float commandA, float commandB){
    
    uint8_t b[2];
    b[0] = u_int8_t((fmax(-1.0, fmin(commandA, 1.0))+1.0)*127.5);
    b[1] = u_int8_t((fmax(-1.0, fmin(commandB, 1.0))+1.0)*127.5);
    
    write(serial_port, b, 2);
//    u_int8_t num_bytes = u_int8_t(read(serial_port, &serialOb, sizeof(serialOb)));
//    SHOW(float(serialOb[0]))
}

}

