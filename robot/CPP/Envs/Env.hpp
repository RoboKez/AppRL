//
//  Env.hpp
//  RL_Bytes
//
//  Created by Kez Smithson Whitehead on 11/03/2022.
//

#ifndef Env_hpp
#define Env_hpp

// Base and IO

#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <array>
#include <string>
#include <vector>
#include <fcntl.h>
#include "Eigen/Dense"
#include <random>

#include<cmath>
#include <cstdint>
#include <algorithm>
#include <iostream>
#include <iomanip>

// Mujoco
#include "mjxmacro.h"
#include "uitools.h"
#include "array_safety.h"

#include <termios.h>

// OpenGL
#include "GLFWRender.hpp"

#define SHOW(x) std::cout << x << std::endl;
#define SHOW2(x, y) std::cout << x << y << std::endl;

struct Quaternion
{
    float w, x, y, z;
};

struct Euler
{
    float roll, pitch, yaw;
};


struct PID
{
    bool firstReading;
    float timestep, E, P, I, D, prevE, prevI, limI;
    PID(float t, float limI = 1);
    ~PID();
    void Reset();
    void Update(float curPoint, float setPoint = 0); // all set prev and stiff
};


class CoreEnv
{
public:
    float m_prev_pitch = 0;
    float m_last_side = 1.0;
    float rad_ratio = 57.2957795131;
    float m_start_pos = 0.0;
    float m_kill_angle = 1.3;
    float m_kill_distance = 1.3;
    float m_sensor_noise = 0.0;
    int m_danger_zone_count;
    
    int m_sym_pairs = 0; 
    
    bool StepLimited = true;
    std::string Name;
    int StepLimit;
    static bool m_quit;
    Eigen::VectorXf m_ob;
    float m_reward = 0;
    uint8_t m_done = 0;
    int step = 0;
    Eigen::VectorXf m_act;
    float m_set_point = 0;
    float m_set_point2 = 0;
    float m_true = 0;
    float m_true2 = 0;
    bool m_watching = false;
    bool m_same_act = false;
    bool m_clock = false;
    
    float ModulusMujoco(float reading, bool inv = false, float modulus = 6.28318530718);
    virtual void MarkovStep(Eigen::VectorXf agent_act) = 0;
    virtual void GetOb() = 0;
    virtual void EpisodeReset(int startPos=0) = 0;
    virtual Eigen::VectorXf NextOb() = 0;
    virtual uint16_t GetObSize() = 0;
    virtual uint16_t GetActSize() = 0;
    virtual ~CoreEnv();
    bool m_deterministic =false;
    float clip(float n, float lower, float upper);
    Quaternion EulerToQuaternion(float roll, float pitch, float yaw);
    Euler QuaternionToEuler(float qw, float qx, float qy, float qz);
    float getTimeset();
    
    
    
protected:
    CoreEnv(std::string & name, int & stepLimit);
//    std::string cur_path = std::filesystem::current_path();
    
    // Env
    int sub_steps;
    int n_sub_steps;
    std::string label;
    float m_tp;
    float m_ts;
    float m_tt;
    
    float m_rp;
    float m_rs;

private:
};


class MjEnv:public CoreEnv
{
public:
    float m_prev_ang = 0;
    GLFWwindow* window;
    mjModel* m = NULL;                  // MuJoCo model
    mjData* d = NULL;                   // MuJoCo data
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context
    mjvPerturb pert;                    // external force
    char error[1000];
    std::default_random_engine m_gen;
    
    void Render();
    virtual void MarkovStep(Eigen::VectorXf agent_act);  //override for hierachical
    virtual ~MjEnv();
    void WarmStart(int steps);
    
protected:
    MjEnv(std::string & name, int & stepLimit);
    virtual void GetOverlay() = 0;
    void SetXmlPath();
    float simstart = 0;
private:
    void InitGlfw();
    std::string xml_path;
};


class RealEnv:public CoreEnv
{
public:
    mjModel* m = NULL;
    int serial_port = 0;
    struct termios tty;
    const char * m_portName;
    int ConnectAgentSerial();
//    int StepLimit;
//    void MarkovStep(Eigen::VectorXf agent_act);
    virtual void MarkovStep(Eigen::VectorXf agent_act) = 0;
    virtual ~RealEnv();
    GLFWwindow* window;
    void Render();
    mjrContext con;
    // custom GPU context
//    std::string Name;
protected:
    RealEnv(std::string & name, int & stepLimit, const char * portName);
private:
    void InitGlfw();
};

#endif /* Env_hpp */
