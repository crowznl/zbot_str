// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#ifndef __Tangair_usb2can__
#define __Tangair_usb2can__

#include <assert.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <signal.h>
#include <string>
#include <cstring>
#include <sys/mman.h>
#include <sys/timerfd.h>
#include <thread>
#include <iomanip>
#include <sstream> 
#include <atomic>
#include <sched.h>
#include <unistd.h>
#include <mutex>
#include <vector>
#include <queue>
#include <eigen3/Eigen/Geometry>
#include "usb_can.h"
// Optional libtorch include; use -DUSE_LIBTORCH OFF when building without LibTorch
#ifdef USE_LIBTORCH
#include <torch/script.h>
#endif


// 辅助函数
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
float getTimestamp();

extern time_t start_tp;

// 灵足电机,此处为RS00参数，其他电机请自行修改或参考电机调试工具上显示的参数
//https://can.robotsfan.com/motor/robstride/desktop.html
#define P_MIN -12.57f
#define P_MAX 12.57f
#define V_MIN -33.0f
#define V_MAX 33.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -14.0f
#define T_MAX 14.0f

#define PI (3.1415926f)

#define Delay_0us 0
#define Delay_75us 75
#define Delay_100us 100
#define Delay_150us 150
#define Delay_300us 300
#define Delay_500us 500
#define Delay_1000us 1000

typedef struct
{
	uint8_t module_id; // 模块ID:0x01~0x0C

	// ********************************************电机部分******************************************** //
	uint8_t master_id;
	uint8_t motor_id; // => 模块ID
	uint8_t fault_message;
	/*
	bit5: HALL 编码故障,10000,16
	bit4: 磁编码故障,01000,8
	bit3: 过温,00100,4
	bit2: 过流,00010,2
	bit1: 欠压故障,00001,1*/
	uint8_t motor_state;
	/*  0 : Reset 模式[复位]
		1 : Cali 模式[标定]
		2 : Motor 模式[运行]*/
	uint8_t mode;

	uint16_t current_position; //[0~65535]对应(-4π~4π)
	uint16_t current_speed;	   //[0~65535]对应(-33rad/s~33rad/s)
	uint16_t current_torque;   //[0~65535]对应(-14Nm~14Nm)
	uint16_t current_temp;	   // 当前温度：Temp(摄氏度）*10

	float current_position_f = 0.0f; //[0~65535]对应(-4π~4π)
	float current_speed_f = 0.0f;	  //[0~65535]对应(-33rad/s~33rad/s)与rpm的换算关系为：1rad/s=9.55rpm
	float current_torque_f;	  //[0~65535]对应(-14Nm~14Nm)
	float current_temp_f;	  // 当前温度：Temp(摄氏度）*10

	float motor_init_position = 0.0f;
	bool motor_is_initialized = false; //电机位置是否初始化标志位（位置初始化：防止电机初始化时转圈）
	bool motor_miscount = false;

	// ********************************************IMU部分******************************************** //

	uint8_t imu_id; // => 模块ID + 0x20
	float quat_w = 1.0f;
	float quat_x = 0.0f;
	float quat_y = 0.0f;
	float quat_z = 0.0f;

} Module_CAN_Recieve_Struct; // zbot模块接收结构体

typedef struct
{
	std::vector<Module_CAN_Recieve_Struct> Module_CAN_Recieve; //*************************************************************** */ 是否有必要通过USB2CAN_Dev_Struct嵌套？忘了当时怎么想的

} USB2CAN_Dev_Struct; // USB转CAN设备接收结构体（包含多个模块的数据）

class Tangair_usb2can
{
public:
	
	bool all_thread_done_;
	bool running_;

	void Spin();

	Tangair_usb2can();

	~Tangair_usb2can();

	std::vector<float> init_angles = {0.312f, 0.837f, -2.02f, 2.02f, -0.837f, -0.312f}; // 给初始角度
	// std::vector<float> init_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 给初始角度
	float a = 0.3f;
	std::vector<float> lower_limit = {0.312f - a*PI, 0.837f - a*PI, -2.02f - a*PI, 2.02f - a*PI, -0.837f - a*PI, -0.312f - a*PI};
	std::vector<float> upper_limit = {0.312f + a*PI, 0.837f + a*PI, -2.02f + a*PI, 2.02f + a*PI, -0.837f + a*PI, -0.312f + a*PI};

	std::vector<float> Q_meas_init = {1.0f, 0.0f, 0.0f, 0.0f}; // IMU初始测量四元数
	Eigen::Quaternionf Q_desired{0.6003f, -0.6003f, -0.3735f, -0.3739f}; // (w, x, y, z) // 注意Eigen中四元数赋值的顺序，实数w在首；但是实际上它的内部存储顺序是[x y z w]
	// Eigen::Quaternionf Q_desired{0.0f, -1.0f, 0.0f, 0.0f}; 
    Eigen::Quaternionf Q_offset{Eigen::Quaternionf::Identity()};

	bool motor_zero_set_already = false;
	
	// CAN设备0
	int USB2CAN0_;
	std::thread _CAN_RX_device_0_thread;
	void CAN_RX_device_0_thread();

    int can_dev0_rx_count;
	int can_dev0_rx_count_thread;

	//can发送测试线程
	std::thread _CAN_TX_thread;
	//can发送测试线程函数
	void CAN_TX_thread();

    // 输出给CAN发送线程的目标位置（由策略线程计算）
	std::vector<float> position_output = init_angles;
	// 发送到电机的弧度
	std::vector<float> motor_angles = init_angles;

	// 线程同步：保护 command_input/position_output 与模型状态
	std::mutex strategy_mutex;

	// 策略线程
	std::thread _strategy_thread;
	void Strategy_thread();

	std::vector<float> out_last = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};

#ifdef USE_LIBTORCH
	// LibTorch 模型（TorchScript）
	std::shared_ptr<torch::jit::script::Module> policy_model;
	bool model_loaded = false;
	// at::Tensor output_tensor = torch::zeros({6});
	at::Tensor relative_tensor = torch::tensor({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
#endif

    //can键盘输入线程
	std::thread _keyborad_input;
	//can发送测试线程函数
	void keyborad_input();
	float command_input = 0.8f*PI;


	// 记录线程
	std::thread _log_thread;
	// 记录日志文件函数
	void Log_thread();
	struct LogFrame {
		float timestamp = 0.0;
		float motor_pos[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		float quat_w = 0.0;
		float quat_x = 0.0;
		float quat_y = 0.0;
		float quat_z = 0.0;
	};

	std::mutex log_rx_mutex;
	std::mutex log_tx_mutex;
	std::mutex log_strategy_mutex;

	std::queue<LogFrame> log_rx_queue;
	std::queue<LogFrame> log_tx_queue;
	std::queue<LogFrame> log_strategy_queue;




	/*********************************************************************************************************************/
	/*****************************************       ***电机及IMU相关***      *******************************************************/
	/*********************************************************************************************************************/

	/************************************************ CAN发送帧相关定义 *****************************************************/
	// 电机基本操作变量
	FrameInfo txMsg_CAN_Motor = {
		.canID = 0,
		.frameType = EXTENDED,
		.dataLength = 8,
	};

	// IMU基本操作变量
	FrameInfo txMsg_CAN_IMU = {
		.canID = 0,
		.frameType = STANDARD,
		.dataLength = 8,
	};

	// 电机扩展CAN帧ID结构体
	typedef struct ID_CAN_Struct{
		uint8_t id;
		uint16_t exdata; // 主机ID
		uint8_t mode;
	} ID_CAN_Struct;

	// 电机ID结构体
	ID_CAN_Struct ID_CAN = {
		.id = 0x01,
		.exdata = 0xfd,
		.mode = 0,
	};

	// 电机运控模式发送结构体
	typedef struct Motor_PDControl_Struct{
		float Feedforward_Torque;
		float Tar_Position;
		float Tar_Velocity;
		float Kp;
		float Kd;
	} Motor_PDControl_Struct;

	Motor_PDControl_Struct Motor1_PDControl = {
		.Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 1.0f,
	};

	Motor_PDControl_Struct Motor2_PDControl = {
		.Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 1.0f,
	};

	Motor_PDControl_Struct Motor3_PDControl = {
		.Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 1.0f,
	};

	Motor_PDControl_Struct Motor4_PDControl = {
		.Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 1.0f,
	};

	Motor_PDControl_Struct Motor5_PDControl = {
		.Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 1.0f,
	};

	Motor_PDControl_Struct Motor6_PDControl = {
		.Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 1.0f,
	};

	// Motor_PDControl_Struct Motor1_PDControl = {
	// 	.Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 0.0f,
	// 	.Kd = 0.0f,
	// };

	// Motor_PDControl_Struct Motor2_PDControl = {
	// 	.Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 0.0f,
	// 	.Kd = 0.0f,
	// };

	// Motor_PDControl_Struct Motor3_PDControl = {
	// 	.Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 0.0f,
	// 	.Kd = 0.0f,
	// };

	// Motor_PDControl_Struct Motor4_PDControl = {
	// 	.Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 0.0f,
	// 	.Kd = 0.0f,
	// };

	// Motor_PDControl_Struct Motor5_PDControl = {
	// 	.Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 0.0f,
	// 	.Kd = 0.0f,
	// };

	// Motor_PDControl_Struct Motor6_PDControl = {
	// 	.Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 0.0f,
	// 	.Kd = 0.0f,
	// };
	
	// CAN帧数据域
	uint8_t Data_CAN[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	/************************************************ CAN接收结构体相关定义 *****************************************************/
	// 统一暂存ID
	uint8_t TEMP_ID = 0;

	// CAN转USB设备-接收数据结构体，每个结构体对应不同接收线程，包含6个模块的电机和IMU数据
	USB2CAN_Dev_Struct DEV0_RX = { std::vector<Module_CAN_Recieve_Struct>(6) }; // 包含ID:1～6 的模块数据

	/****************************************************** 函数定义 ***********************************************************/
	void Motor_Enable(int32_t dev, uint8_t channel, uint32_t motor_id);

	void Motor_Disable(int32_t dev, uint8_t channel, uint32_t motor_id);

	void ENABLE_ALL_Motor(int delay_us);

	void DISABLE_ALL_Motor(int delay_us);

	void Motor_Initpos_Get(int32_t dev, uint8_t channel, uint32_t motor_id);

	void Motor_Mode_Change(int32_t dev, uint8_t channel, uint32_t motor_id , uint8_t mode);

	void Motor_Zero_Set(int32_t dev, uint8_t channel, uint32_t motor_id);

	void Motor_Zero_Set_ALL(int delay_us);

	void Init_Motor(int32_t dev, uint8_t channel, uint32_t motor_id);

	void Motor_PD_Control(int32_t dev, uint8_t channel, uint32_t motor_id, Motor_PDControl_Struct *Motor_PDControl, float position);

	void ALL_Motor_PD_Control(int delay_us, std::vector<float> motor_angles);

	void Init_Motor_ALL(int delay_us);

	void IMU_Set_SYNC_Mode(int32_t dev, uint8_t channel, uint8_t imu_id);

	void IMU_Set_ALL_SYNC_Mode(int delay_us);

	void IMU_Send_SYNC(int delay_us);

	void IMU_Get_init_quat();

	void IMU_Get_offset_quat();

	void IMU_quat_correct();

private:


};

#endif

// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
