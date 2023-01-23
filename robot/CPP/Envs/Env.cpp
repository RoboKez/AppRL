//
//  Env.cpp
//  RL_Bytes
//
//  Created by Kez Smithson Whitehead on 11/03/2022.
//

#include "Env.hpp"

bool CoreEnv::m_quit;

/*-------------- BaseEnv--------------*/
CoreEnv::CoreEnv(std::string & name, int & stepLimit)
:Name(name), StepLimit(stepLimit){
    m_quit = false;
    SHOW("Core Env init")
    StepLimit = 200;
    
}

float CoreEnv::ModulusMujoco(float reading, bool inv, float modulus){
    float mult = 1000;
    int tmp1 = reading*mult;
    if (tmp1 < 0){
        tmp1 = modulus - abs(tmp1)%(int(modulus*mult));
    } else {
        tmp1 = tmp1%(int(modulus*mult));
    }
    float out = float(tmp1)/(modulus*mult) - 0.5;
    out = out * -2.0;
    if (out>1.0){
        out -= 2.0;
    }
    if(inv){out = out*-1.0;}
    
    if (out<0) {
        out += 1.0;
    } else {
        out -= 1.0;
    }
    return out;
}


Quaternion CoreEnv::EulerToQuaternion(float roll, float pitch, float yaw){
    roll /= rad_ratio;
    pitch /= rad_ratio;
    yaw /= rad_ratio;

    
    // Abbreviations for the various angular functions
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

Euler CoreEnv::QuaternionToEuler(float qw, float qx, float qy, float qz){
    Quaternion q;
    q.w = qw;
    q.x = qx;
    q.y = qy;
    q.z = qz;
    
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    
    Euler e;
    e.roll = std::atan2(sinr_cosp, cosr_cosp);
    e.yaw = std::atan2(siny_cosp, cosy_cosp);
    if (std::abs(sinp) >= 1)
        e.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        e.pitch = std::asin(sinp);
    
    e.roll *= rad_ratio;
    e.pitch *= rad_ratio;
    e.yaw *= rad_ratio;
    return e;
}


PID::PID(float timestep_, float limI_)
:timestep(timestep_), limI(limI_){
    SHOW2("pid struct init with timestep: ", timestep)
}

void PID::Reset(){
    E = 0;
    P = 0;
    I = 0;
    D = 0;
    
    prevE = 0;
    prevI = 0;
    
    firstReading = true;
//    SHOW("PID reset")
}

void PID::Update(float curPoint, float setPoint){
    E = setPoint - curPoint;
    P = E;
    I = (E + prevE) * timestep * 0.5f + prevI;
    D = (E - prevE) / timestep;
    if(firstReading){D = 0; firstReading = false;} // assumes system starts stntionary
    
    I = std::max(-limI, std::min(I, limI));
    
    prevE = E;
    prevI = I;
    
//    SHOW("PI forward, prevs updated")
}

PID::~PID()
{
    SHOW("PID destroyed");
}

CoreEnv::~CoreEnv()
{
    SHOW("Core destroyed");
}


/*-------------- MjEnv--------------*/

MjEnv::MjEnv(std::string & name, int & stepLimit)
:CoreEnv(name, stepLimit)
{
    SHOW2("Name:\t", name);
    SetXmlPath();
    m = mj_loadXML(xml_path.c_str(), 0, error, 1000);


    
    if( !m ) {
        printf("%s\n", error);
        m_quit = true;
        SHOW("RL Error: Can't find xml")
    }
    d = mj_makeData(m); // make data
    
    n_sub_steps =  10;
    m_rp = 5.0f;
    m_rs =  5.0f;
    
    m_tp = m->opt.timestep*n_sub_steps;
    m_ts = m->opt.timestep*n_sub_steps*m_rp;
    m_tt = m->opt.timestep*n_sub_steps*m_rp*m_rs;
    
    InitGlfw();
    
}

void MjEnv::SetXmlPath()
{
    if(Name=="cartpoleP" || Name=="cartpolePI" || Name=="cartpolePD" || Name=="cartpolePID" ||  Name=="cartpoleClassicI" || Name=="cartpoleClassic" || Name=="Classic_Primary"){
        xml_path = "cartpole.xml";
        m_same_act = false;
    } else if(Name=="piperP" || Name=="piperPVPV" || Name=="piperPD" || Name=="piperPID" || Name=="piperClassic"){
        xml_path = "piper.xml";
        m_same_act = true;
    }else if (Name=="piperAll"){
        xml_path = "piper.xml";
        m_same_act = false;
    }else if (Name=="link_clock"){
        xml_path = "link_unlimited.xml";
        m_same_act = false;
    }else if (Name=="link_symmetry" || Name=="link_none" || Name=="MAPASgallop" || Name=="MAPAScrawl" || Name=="MAPAStrot" || Name=="PASgallop" || Name=="PAScrawl" || Name=="PAStrot"){
        xml_path = "link_none.xml";
        m_same_act = false;
    } else {
        xml_path = Name +".xml";
        m_same_act = false;
    }
    std::cout << "Set xml path: " << xml_path << std::endl;
}

void MjEnv::InitGlfw(){
    
    if (GLFWStuff::windowCount == 0){
        if (!glfwInit())
            SHOW("Error: NO GLFW");
        SHOW2("init for window count is", GLFWStuff::windowCount)
    }
    window = glfwCreateWindow(800, 500, Name.c_str(), NULL, NULL);
    GLFWStuff::windowCount += 1;
    SHOW2("window created for ", xml_path);

    if (!window)
    {
        glfwTerminate();
        SHOW("No glfw window");
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    glfwSetKeyCallback(window, GLFWStuff::KeyboardCallback);
    glfwSetCursorPosCallback(window, GLFWStuff::MouseMoveCallback);
    glfwSetMouseButtonCallback(window, GLFWStuff::MouseButtonCallback);
    glfwSetScrollCallback(window, GLFWStuff::ScrollCallback);
}

void MjEnv::Render()
{
    if(glfwWindowShouldClose(window)) {
        m_quit = true;
        SHOW("glfw quitting");
    
    } else if (d->time - simstart > float(GLFWStuff::windowCount)/GLFWStuff::framerate or (m_done and !m_watching)){

        glfwMakeContextCurrent(window);
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        GetOverlay();
        if(GLFWStuff::help){
            label = "'h' = help screen\n" + std::to_string(int(d->time)) + "s";
            
        }
        mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, viewport, label.c_str(), NULL, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
        simstart = d->time;
    }
}

void MjEnv::WarmStart(int steps) {
    Eigen::VectorXf rand_acts(GetActSize());
    for(int i=0; i<steps; i++){
        step = -1;
        for(int ii=0; ii<GetActSize(); ii++){
            std::uniform_int_distribution<> uni_dist(-1000, 1000);
            rand_acts[ii] = float(uni_dist(m_gen))/1000.0f;
        }
        MarkovStep(rand_acts);
    }
}


void MjEnv::MarkovStep(Eigen::VectorXf agent_act)
{
    
    for(int i=0; i<agent_act.size(); i++){
        d->ctrl[i] = agent_act[i];
    }
    if(m_same_act){
        d->ctrl[1] = agent_act[0];
        d->ctrl[0] += GLFWStuff::sp*0.1;
        d->ctrl[1] -= GLFWStuff::sp*0.1;
    }
    
    sub_steps = 0;
    while (sub_steps < n_sub_steps and !m_quit){
        mj_step(m, d);
        sub_steps+=1;
        cam.fixedcamid =0;
        cam.trackbodyid = 3;
        cam.type = GLFWStuff::camView;
        if (GLFWStuff::ScrollCam){
            GLFWStuff::ScrollCam = false;
            mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*GLFWStuff::ScrollCam_yoffset, &scn, &cam);
        }
        if (GLFWStuff::MoveCam){
            GLFWStuff::MoveCam = false;
            mjv_moveCamera(m, GLFWStuff::mouse_action, GLFWStuff::x_h, GLFWStuff::y_h, &scn, &cam);
        }
        if(GLFWStuff::renderAll){Render();}
    }
//    Render();
    if(!GLFWStuff::renderAll){Render();}
    step +=1;
    GetOb(); // Get next observation, rewards and done
    
    if(m_done)Render();
    
}

MjEnv::~MjEnv()
{
    SHOW("MjEnv destroyed");
    //free visualization storage
      mjv_freeScene(&scn);
      mjr_freeContext(&con);

      // free MuJoCo model and data
      mj_deleteData(d);
      mj_deleteModel(m);
      glfwTerminate();
    GLFWStuff::windowCount = 0;
}




/*-------------- RealEnv--------------*/

RealEnv::RealEnv(std::string & name, int & stepLimit, const char * portName)
:CoreEnv(name, stepLimit), m_portName(portName)
{
    SHOW2("Real Name:\t", name);
    InitGlfw();
}

void RealEnv::InitGlfw(){
    SHOW("start")
    if (!glfwInit())
        SHOW("Error: NO GLFW");
    SHOW("start")
    
    window = glfwCreateWindow(400, 200, Name.c_str(), NULL, NULL);
    SHOW("window created");

    if (!window)
    {
        glfwTerminate();
        SHOW("No glfw window");
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    glfwSetKeyCallback(window, GLFWStuff::KeyboardCallback);
    glfwSetCursorPosCallback(window, GLFWStuff::MouseMoveCallback);
    glfwSetMouseButtonCallback(window, GLFWStuff::MouseButtonCallback);
    glfwSetScrollCallback(window, GLFWStuff::ScrollCallback);
}

void RealEnv::Render()
{
    if(glfwWindowShouldClose(window)) {
        m_quit = true;
        SHOW("glfw quitting");
    } else{
        glClear(GL_COLOR_BUFFER_BIT);
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
//        GetOverlay();
        mjr_overlay(mjFONT_BIG, mjGRID_TOPLEFT, viewport, label.c_str(), NULL, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

RealEnv::~RealEnv()
{
    SHOW("RealEnv destroyed");
    glfwTerminate();
}


int RealEnv::ConnectAgentSerial() {
    
    serial_port = open(m_portName, O_RDWR);
    SHOW2("Port Name:\t", m_portName);
    SHOW2("Serial Port:\t", serial_port);
    
    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return -1;
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
        return -1;
    }
    return 1;
}

float CoreEnv::clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

float CoreEnv::getTimeset(){
    return m_tp;
}
