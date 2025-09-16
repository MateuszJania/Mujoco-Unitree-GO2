#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);


#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>

int mode = 0;

int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    
    ch = getchar();
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    
    return 0;
}

void KeyboardThread()
{
    while (true)
    {
        if (kbhit())
        {
            char c = getchar();
            if (c == 'w')
            {
                mode = 1;
                std::cout << "[Key] W pressed \n";
            }
            else if (c == 's')
            {
                mode = 2;
                std::cout << "[Key] S pressed \n";
            }
              else if (c == 'd')
            {
                mode = 3;
                std::cout << "[Key] D pressed \n";
            }
           else if (c == 'a')
            {
                mode = 4;
                std::cout << "[Key] A pressed \n";
            }
           else if (c == 't')
            {
                mode = 5;
                std::cout << "[Key] T pressed \n";
            }
      else if (c == 'g')
            {
                mode = 6;
                std::cout << "[Key] G pressed \n";
            }
        }
        usleep(100000); // 100 ms
    }
}



class Custom
{
public:
    Custom(){};
    ~Custom(){};
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

private:
   // double stand_up_joint_pos[12] = {0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    //                                 0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763};
   //double stand_down_joint_pos[12] = {0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 
    //                                   0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375};
                                       
     double stand_up_joint_pos[12] = {0, 1.2, -1.3, 0, 1.2, -1.3,        //prawy przód, lewy przód, prawy tył, lewy tył
                                     0, 1.2 , -1.2, 0, 1.2, -1.2};      // przód góra, tył dół                               
   double stand_down_joint_pos[12] = {0, 1.2, -2.4, 0, 1.2, -2.4,
                                    0, 1.2, -2.4, 0, 1.2, -2.4};   
                                    
      double walk_1_joint_pos[12] = {0, 0.6, -1.5, 0, 0.3, -1.3,
                                     0, 0.3, -1.2, 0, 0.6, -1.4}; 
                                     
      double walk_2_joint_pos[12] = {0, 0.3, -1.2, 0, 0.6, -1.5,
                                     0, 0.6, -1.4, 0, 0.3, -1.2};                          
                                    
                                    
                                    
    double turn_right_joint_pos[12] = {0, 1.2, -2.0, 0, 1.2, -2.4,
                                    0, 1.2, -2.4, 0, 1.2, -2.4};   
                                    
     double turn_left_joint_pos[12] = {0, 1.2, -2.4, 0, 1.2, -2.0,
                                    0, 1.2, -2.4, 0, 1.2, -2.4};                                 
                                       
    double dt = 0.002;
    double runing_time = 0.0;
    double phase = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};     // default init
    unitree_go::msg::dds_::LowState_ low_state{}; // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Custom::Init()
{
    InitLowCmd();
    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &Custom::LowCmdWrite, this);
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01); // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Custom::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void Custom::LowCmdWrite()
{

if (mode == 1)
{

    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = stand_up_joint_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = 70;
        low_cmd.motor_cmd()[i].kd() = 3.5;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}
else if (mode == 2)
{
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = stand_down_joint_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = 70;
        low_cmd.motor_cmd()[i].kd() = 3.5;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}
else if (mode ==3)
{
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = turn_right_joint_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = 70;
        low_cmd.motor_cmd()[i].kd() = 3.5;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}
else if (mode ==4)
{
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = turn_left_joint_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = 70;
        low_cmd.motor_cmd()[i].kd() = 3.5;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}
else if (mode ==5)
{
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = walk_1_joint_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = 70;
        low_cmd.motor_cmd()[i].kd() = 3.5;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}
else if (mode ==6)
{
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = walk_2_joint_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = 70;
        low_cmd.motor_cmd()[i].kd() = 3.5;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}



    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
}


int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }
    std::cout << "Press enter to start";
    std::cin.get();
    Custom custom;
    custom.Init();

    std::thread keyThread(KeyboardThread);
    keyThread.detach();  // lub join jeśli chcesz czekać

    while (1)
    {
        sleep(10);
    }

    return 0;
}