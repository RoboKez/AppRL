//
//  EnvTrolleyOuter.hpp
//  robot
//
//  Created by Kez Smithson Whitehead on 23/05/2022.
//

#ifndef EnvTrolleyOuter_hpp
#define EnvTrolleyOuter_hpp

#include  "Env.hpp"

#include <stdio.h>

class TrolleyEnvOuter:public RealEnv
{
public:
    Eigen::VectorXf m_ob = Eigen::VectorXf(2);  //ob size
    std::uint16_t m_act_size = 1;               // act size
    
    TrolleyEnvOuter(std::string name, const char * portName, int m_seed=0, int step_limit=200);
    int ConnectAgentSerial();
    void DisconnetAgentSerial();
    void EpisodeReset();
    void GetOb();
    void MarkovStep(Eigen::VectorXf agent_act);
    Eigen::VectorXf NextOb();
    uint16_t GetObSize();
    uint16_t GetActSize();
    ~TrolleyEnvOuter();
    
    int serial_port = 0;
    struct termios tty;
    const char * m_portName;
    u_int8_t packetID;
    
protected:
    std::string m_name = "trolley";
    void TargetReset();
private:
    bool m_target_moved = true;
    float m_old_pole_pos;
    float m_kill_angle = 0.5;
};

#endif /* EnvTrolley_hpp */
