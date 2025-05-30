#include "motor_cfg.h"

void RobStrideMotor::init_socket()
{
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0)
    {
        perror("socket");
        exit(1);
    }

    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ);
    if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("ioctl");
        exit(1);
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        exit(1);
    }
}
void RobStrideMotor::Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode)
{
    struct can_frame frame{};
    
    frame.can_id = Communication_Type_SetSingleParameter<<24|master_id<<8|motor_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 0x08;

    frame.data[0] = Index;
    frame.data[1] = Index>>8;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    
	if (Value_mode == 'p')
	{
		memcpy(&frame.data[4],&Value,4);
	}
	else if (Value_mode == 'j')
	{
		// Motor_Set_All.set_motor_mode = int(Value);
		frame.data[4] = (uint8_t)Value;
		frame.data[5] = 0x00;	
		frame.data[6] = 0x00;	
		frame.data[7] = 0x00;	
	}
    int n = write(socket_fd, &frame, sizeof(frame));
    if (n != sizeof(frame))
    {
        perror("set mode failed");
    }
    else
    {
        std::cout << "[✓] Motor set-mode command sent." << std::endl;
    }
}
// 发送使能指令（通信类型3）
void RobStrideMotor::enable_motor()
{
    struct can_frame frame{};
    frame.can_id = (Communication_Type_MotorEnable<<24)|(master_id<<8)|motor_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 8;
    memset(frame.data, 0, 8);

    int n = write(socket_fd, &frame, sizeof(frame));
    if (n != sizeof(frame))
    {
        perror("enable_motor failed");
    }
    else
    {
        std::cout << "[✓] Motor enable command sent." << std::endl;
    }
}

uint16_t RobStrideMotor::float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (x < x_min)
        x = x_min;
    if (x > x_max)
        x = x_max;
    float span = x_max - x_min;
    float offset = x - x_min;
    return static_cast<uint16_t>((offset * ((1 << bits) - 1)) / span);
}

// 发送运控模式（控制角度 + 速度 + KP + KD）
void RobStrideMotor::send_motion_command(float torque,
                                         float position_rad,
                                         float velocity_rad_s,
                                         float kp,
                                         float kd)
{
    struct can_frame frame{};
    frame.can_id = (Communication_Type_MotionControl << 24) | (float_to_uint(torque,-17.0f,17.0f,16)<<8) | motor_id;
    frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    // frame.can_id = 0x1200fd01;
    frame.can_dlc = 8;

    uint16_t pos = float_to_uint(position_rad, -4 * M_PI, 4 * M_PI, 16);
    uint16_t vel = float_to_uint(velocity_rad_s, -44, 44, 16);
    uint16_t kp_u = float_to_uint(kp, 0, 500, 16);
    uint16_t kd_u = float_to_uint(kd, 0, 5, 16);

    frame.data[0] = (pos >> 8);
    frame.data[1] = pos;
    frame.data[2] = (vel >> 8);
    frame.data[3] = vel;
    frame.data[4] = (kp_u >> 8);
    frame.data[5] = kp_u;
    frame.data[6] = (kd_u >> 8);
    frame.data[7] = kd_u;
    // 05 70 00 00 07 01 82 F9

    int n = write(socket_fd, &frame, sizeof(frame));

    if (n != sizeof(frame))
    {
        perror("send_motion_command failed");
    }
}

void RobStrideMotor::send_velocity_mode_command(float velocity_rad_s)
{
    struct can_frame frame{};
    frame.can_id = (0x02<<8) | motor_id;
    // frame.can_id |= CAN_EFF_FLAG; // 扩展帧
    frame.can_dlc = 8;

    float2uchar v;
    float2uchar I;
    v.f = velocity_rad_s;
    I.f = 27.0f;

    frame.data[0] = v.c[0];
    frame.data[1] = v.c[1];
    frame.data[2] = v.c[2];
    frame.data[3] = v.c[3];
    frame.data[4] = I.c[0];
    frame.data[5] = I.c[1];
    frame.data[6] = I.c[2];
    frame.data[7] = I.c[3];

    int n = write(socket_fd, &frame, sizeof(frame));
    if (n != sizeof(frame))
    {
        perror("send_motion_command failed");
    }

}


float RobStrideMotor::read_initial_position()
{
    struct can_frame frame{};
    auto start = std::chrono::steady_clock::now();
    while (true)
    {
        // float neutral_pos = 2.0f;
        // send_motion_command(neutral_pos, 0.0f, 0.0f, 0.0f);

        ssize_t nbytes = read(socket_fd, &frame, sizeof(frame));
        if (nbytes > 0 && (frame.can_id & CAN_EFF_FLAG))
        {
            uint32_t canid = frame.can_id & CAN_EFF_MASK;
            uint8_t type = (canid >> 24) & 0xFF;
            uint8_t mid = (canid >> 8) & 0xFF;
            uint8_t eid = canid & 0xFF;

            // please switch print output;
            printf("type: 0x%02X\n", type);
            printf("mid:  0x%02X\n", mid);
            printf("eid:  0x%02X\n", eid);

            if (type == 0x02 && mid == 0x01 && eid == 0xFD)
            {
                uint16_t p_uint = (frame.data[0] << 8) | frame.data[1];
                float pos = uint_to_float(p_uint, -4 * M_PI, 4 * M_PI, 16);
                std::cout << "[✓] Initial position read: " << pos << " rad" << std::endl;
                return pos;
            }
        }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() > 10000)
        {
            std::cerr << "[!] Timeout waiting for motor feedback." << std::endl;
            return 0.0f;
        }
    }
}
