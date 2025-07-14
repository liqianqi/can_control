# 配置can端口
```shell
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

sudo ip link set can0 type can bitrate 1000000 

sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 100
```
- 灵足时代的串口转CAN模块只适用于灵足时代的上位机，Linux上无法使用(除非解析串口数据，太麻烦)，所以在Linux上使用can-able设备，无需slcan
- can0端口是一个示例，实际端口按自己的为准，可能是can1、can2等等。
- 过程中可以使用这个命令查看can端口状态
- 注意第五行的id，电机默认id是0x7F(127)
- 电机每发送一次命令就会返回一帧数据，数据包含当前电机位置，速度，扭矩，温度信息
```shell
ip -d link show can0    (can0换成自己的端口)
```
- ERROR-ACTIVITY是正常的, ERROR-PASSIVE以及BUS-OFF是非正常的。

# 编译运行
```shell
mkdir build
cd build
cmake ..
make 
./can_control
```
# 简单说明
- 代码目前默认是速度模式，改成运控模式把第8行和29行注释掉，把7行和28行的注释取消。
- 更多功能正在更新。

