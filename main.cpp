#include "motor_cfg.h"

int main(int argc, char **argv)
{
    RobStrideMotor motor("can0", 0xFF, 0x01);

    // motor.Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);		//设置电机模式
    motor.Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);		//设置电机模式

    motor.enable_motor(); // 1. 使能电机

    sleep(1);

    float position = 1.57f;
    float velocity = 0.1f;

    float frequency = 1.0;  // 频率为 1 Hz
    float amplitude = 0.2;  // 振幅为 1 rad

    // float dt = 0.02f;
    auto init = std::chrono::system_clock::now();
    while (true)
    {
        // auto now = std::chrono::system_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - init);

        // position = sin(2 * M_PI * frequency * duration.count()) * amplitude;
        // auto [position_feedback, velocity_feedback, torque, temperature] = motor.send_motion_command(0.0, position, velocity, 2.3f, 0.3f);
        auto [position_feedback, velocity_feedback, torque, temperature] = motor.send_velocity_mode_command(5.0f);
        std::cout << "position_feedback: " << position_feedback << std::endl;
        std::cout << "velocity_feedback: " << velocity_feedback << std::endl;
        std::cout << "torque: " << torque << std::endl;
        std::cout << "temperature: " << temperature << std::endl;
        // std::cout << "position: " << position << std::endl;
        usleep(100000); // 每次发送后暂停1毫秒
    }
    // motor.send_motion_command(1.0, position, velocity, 1.0f, 0.1f);

    return 0;
}
