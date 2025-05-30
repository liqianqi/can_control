# 配置can端口
```shell
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

sudo ip link set can0 type can bitrate 1000000 

sudo ip link set can0 up
sudo ifconfig can0 txqueuelen 100
```

- can0端口是一个示例，实际端口按自己的为准，可能是can1、can2等等。
- 过程中可以使用这个命令查看can端口状态
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

