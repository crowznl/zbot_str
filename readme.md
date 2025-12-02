

# 简介
   本程序使用C++在Linux环境编写、运行，通过USB2CAN模块使Linux系统电脑拓展两路CAN总线，每条总线挂载3个灵足RS00电机，程序可以以约500hz的频率控制6个灵足RS00电机和一个IMU，用于机器人策略部署。


# 前期准备
## 0. Libtorch
 由于通信程序采用C++,本程序使用Libtorch进行策略部署，请安装Libtorch。
## 1. 硬件准备
本程序需要与肥猫机器人公司USB2CAN模块配合使用，请准备好模块与模块说明书、模块SDK，`并按照说明书使用install.sh文件安装USB2CAN规则文件，或手动安装规则文件，安装方法：`
1. 进入项目目录下的can文件夹
```bash
cd USB2CAN-Demo-Lingzu/can
```
2. 复制规则文件usb_can.rules 到/etc/udev/rules.d/
```bash
sudo cp usb_can.rules /etc/udev/rules.d/
```
3. 运行下面的命令，使udev规则生效
```bash
sudo udevadm trigger
```

```USB转2路CAN模块购买地址：```
https://e.tb.cn/h.TBC18sl6EZKXUjL?tk=C5g5eLgyMf6HU071

```说明书以及SDK下载地址：```
https://pan.baidu.com/s/1EwYDNQ0jMKyTSvJEEcj6aw?pwd=10ob




# 安装
1. 克隆仓库到本地 :
```bash
git clone https://github.com/crowznl/zbot_str.git
```
2. 进入项目目录 :
```bash
cd zbot_str
```
3. 修改依赖项 :
```bash
vim CMakeLists.txt
# "/home/<your-path>/libtorch"
vim src/Tangir_usb2can.cpp
# Tangair_usb2can::Tangair_usb2can()
# policy_model = std::make_shared<torch::jit::script::Module>(torch::jit::load("/home/<your-path>/policy.pt"));
```
4. 编译项目 :
```bash
mkdir build
cd build
cmake ..
make -j4
```
5. 运行项目 :
```bash
./can_code
```


# 注意事项
1. 本程序使用一个USB2CAN模块（设备名称为USB2CAN0）的两个总线channel，分别挂载3个电机。
2. 若还需要拓展多个USB2CAN模块，可参考[上游仓库程序](https://github.com/SOULDE-Studio/USB2CAN-Demo-Lingzu)的基础上进行修改，一个电脑最多拓展4个模块即8路CAN总线。
3. 在同一模块的同一条CAN总线发送的控制命令间隔不应小于300us，可以交错发送不同CAN总线上的控制命令。
4. 程序封装了电机数据结构体，只需要对结构体对象赋值再调用发送函数，即可控制电机，赋值数值范围请参考灵足电机说明书
5. 本程序使用灵足RS00电机，如使用其他型号电机请修改头文件参数，[参考网站](https://can.robotsfan.com/motor/robstride/desktop.html)。
6. 本程序主要由发送线程、接收线程、策略线程、键盘输入线程和log线程组成。
7. 电机CAN口的H和L与USB2CAN模块CAN口的H和L要对应连接，高对高，低对低。
8. 经测试，对于单个电机，控制命令发送绝对时间间隔为282微秒时，有效带宽（即发送一帧控制命令，可收到一帧回复）达到最大3509.78赫兹，丢包率为0.9%


# 引用说明

- 上游仓库简介
   
程序使用C++在Linux环境编写、运行，通过两个USB2CAN模块使Linux系统电脑拓展4路CAN总线，每条总线挂载3个灵足RS04电机，程序可以以1000hz的频率控制共计12个灵足RS04电机，适用于一般四足机器人控制底层。
