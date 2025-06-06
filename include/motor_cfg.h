#include <iostream>
#include <cstring>
#include <cmath>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <ctime>

#define Set_mode 	  'j' //设置控制模式
#define Set_parameter 'p' //设置参数
//各种控制模式
#define move_control_mode  0 //运控模式
#define Pos_control_mode   1 //位置模式
#define Speed_control_mode 2 //速度模式
#define Elect_control_mode 3 //电流模式
#define Set_Zero_mode      4 //零点模式

//通信地址
#define Communication_Type_Get_ID 0x00  //获取设备的ID和64位MCU唯一标识符`
#define Communication_Type_MotionControl 0x01  //运控模式用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02  //用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03  //电机使能运行
#define Communication_Type_MotorStop 0x04  //电机停止运行
#define Communication_Type_SetPosZero 0x06  //设置电机机械零位
#define Communication_Type_Can_ID 0x07  //更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12  //设置电机模式
#define Communication_Type_GetSingleParameter 0x11  //读取单个参数
#define Communication_Type_SetSingleParameter 0x12  //设定单个参数
#define Communication_Type_ErrorFeedback 0x15  //故障反馈帧

typedef union
{
    float f;
    unsigned char c[4];
}float2uchar;

class RobStrideMotor
{
public:
    RobStrideMotor(const std::string can_interface, uint8_t master_id, uint8_t motor_id)
        : iface(can_interface), master_id(master_id), motor_id(motor_id)
    {
        init_socket();
    }

    ~RobStrideMotor()
    {
        if (socket_fd >= 0)
            close(socket_fd);
    }

    void Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode);

    void send_velocity_mode_command(float velocity_rad_s);

    // 发送使能指令（通信类型3）
    void enable_motor();

    float read_initial_position();

    void init_socket();

    uint16_t float_to_uint(float x, float x_min, float x_max, int bits);
    // 发送运控模式（控制角度 + 速度 + KP + KD）
    void send_motion_command(float torque,
                             float position_rad,
                             float velocity_rad_s,
                             float kp = 0.5f,
                             float kd = 0.1f);

    float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        return ((float)x_int) * span / ((1 << bits) - 1) + x_min;
    }

private:
    std::string iface;
    uint8_t master_id;
    uint8_t motor_id;
    int socket_fd = -1;
};
