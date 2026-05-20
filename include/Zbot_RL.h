#ifndef __Zbot_RL__
#define __Zbot_RL__

#include "RS_motor.h"
#include "Hipnuc_IMU.h"
#include <atomic>
#include <queue>
#include <mutex>
#include <unistd.h> 
#include <thread>
#include <vector>
#include <fstream> 
#include <iomanip> 
#include <iostream>
#include <algorithm>
#include <chrono>
#include <signal.h>
// Optional libtorch include; use -DUSE_LIBTORCH OFF when building without LibTorch
#ifdef USE_LIBTORCH
#include <torch/script.h>
#endif

#define PI (3.1415926f)

#define Delay_0us 0
#define Delay_75us 75
#define Delay_100us 100
#define Delay_150us 150
#define Delay_300us 300
#define Delay_500us 500
#define Delay_1000us 1000
#define Delay_10000us 10000

#define PD_MODE 0
#define PP_MODE 1

extern std::atomic<bool> g_stop; // 全局停止标志

// 读取csv文件：返回二维float向量；若读取/解析出错则 load_ok=false  // 现在看好像搞那个布尔引用有点多余了，直接判断return 空 就行了。
// 注意：这是一个全局函数，不是类成员函数；observation_space 是类内常量，不能在该函数内部直接访问，需要从调用处显式传入。
std::vector<std::vector<float>> loadCSV_invec(const std::string& path, bool& load_ok, int expected_cols);

typedef struct
{
	uint8_t module_id; // 模块ID
	
    RS_Motor_Struct Motor_Recieve; // 电机部分

	Hipnuc_IMU_Struct IMU_Recieve; // IMU部分

} Module_CAN_Recieve_Struct; // zbot模块接收结构体

typedef struct
{
	std::vector<Module_CAN_Recieve_Struct> Module_CAN_Recieve; 

} USB2CAN_Dev_Struct; // USB转CAN设备接收结构体（包含多个zbot模块的数据）

class Zbot_RL
{ 
public:

    void Spin();

	Zbot_RL();

	~Zbot_RL();

    void ALL_Motor_ENABLE(int delay_us);

	void ALL_Motor_DISABLE(int delay_us);

    void ALL_Motor_PP_Mode_Set(int delay_us);

    void ALL_Motor_PP_Angle_Set(int delay_us, std::vector<float> motor_angles);

	void ALL_Motor_PP_Init(std::vector<float> motor_angles);

    void ALL_Motor_Zero_Set(int delay_us);

	void ALL_Motor_Angle_Read(int delay_us);

    void ALL_Motor_PD_Init(std::vector<float> motor_angles);

	void ALL_Motor_PD_Control(int delay_us, std::vector<float> motor_angles);

    // 清理接收缓存函数
	void Read_Clear(int dev, int num);

	// 获取当前时间戳
	float getTimestamp();

private:

	// ************************************************ 时间基准************************************************ //
	std::chrono::steady_clock::time_point start_tp_;

    // ************************************************ 线程标志位 ************************************************ //
	
	// bool all_thread_done_;
	bool running_;

	// ************************************************ USB2CAN设备 ************************************************ //
	
	int USB2CAN0_;

	// ************************************************ 初始化参数 ************************************************ //

	// 编译期常量：需要改维度时，改这里并重新编译
	static constexpr bool motor_zero_set_already = true; // 注意是否进行过零点设置
	static constexpr int Motor_Ctrl_Mode = PD_MODE; // 选择电机控制模式
	static constexpr int motor_dof = 6; //8 // 电机数量
	static constexpr int action_space = motor_dof; // 策略输出维度 // 类CPG motor_dof * 3
	static constexpr bool obs_include_command = true;
	static constexpr bool obs_include_heading_err = false;
	static constexpr int observation_space =
	    4 + motor_dof + motor_dof + action_space + (obs_include_command ? 1 : 0) + (obs_include_heading_err ? 1 : 0);

	static constexpr bool sim_obs_play = false; // 是否为仿真观测输入模式，可用于对比实际得到的obs（理论上和sim_action_play的效果一样，因为pytorch和libtorch的结果是一致的）
	std::string sim_obs_csv = "../figures_data/data/obs_env_csv1219/v2_obs.csv";
	// 读取csv_invec相关变量
	std::vector<std::vector<float>> csv_data;
	int csv_index = 0;

	// ************************************************ 日志配置 ************************************************ //
	static constexpr bool log_enabled = false; // 是否启用日志线程
	static constexpr char log_file_path[] = "../figures_data/data/"; // 日志路径
	enum LogType {LOG_TX, LOG_RX, LOG_STRATEGY};
	static constexpr LogType log_data_type = LOG_STRATEGY; // 记录类型

#ifdef USE_LIBTORCH
	// LibTorch 模型（TorchScript）
	std::shared_ptr<torch::jit::script::Module> policy_model;
	bool model_loaded = false;
	// std::string model_path = "/home/rain/libtorch/export_model/policy_standupsymmetry0.5.pt";
	// std::string model_path_standupre = "/home/rain/libtorch/export_model/standup/resymmetry_policy.pt";  // 0119
	std::string model_path_standup = "/home/rain/libtorch/export_model/standup/policy.pt";  // 0121
	std::string model_path_biped = "/home/rain/libtorch/export_model/biped_keyboard.pt";
	std::string model_path_snake = "/home/rain/libtorch/export_model/snake.pt";

	float joint_speed_limit = 1.0f; // 电机速度限制，固定为1(pi rad/s)，不再通过键盘输入修改。

	// at::Tensor relative_tensor = torch::zeros({action_space}, torch::kFloat32);
	at::Tensor relative_tensor = torch::tensor({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
#endif
	
	// std::vector<float> zero_angles = std::vector<float>(motor_dof, 0.0f);
	std::vector<float> zero_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 初始角度--摆直
	// std::vector<float> init_angles = {0.312f, 0.837f, -2.02f, 2.02f, -0.837f, -0.312f}; // 初始角度--站立
	std::vector<float> init_angles = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	// float range = 0.3f * PI;
	// std::vector<float> lower_limit = {0.312f - range, 0.837f - range, -2.02f - range, 2.02f - range, -0.837f - range, -0.312f - range}; // 电机运行范围
	// std::vector<float> upper_limit = {0.312f + range, 0.837f + range, -2.02f + range, 2.02f + range, -0.837f + range, -0.312f + range}; // 电机运行范围
	float range = 1.0f * PI;
	std::vector<float> lower_limit = std::vector<float>(motor_dof, -range); // 电机运行范围
	std::vector<float> upper_limit = std::vector<float>(motor_dof, range); // 电机运行范围

	std::vector<float> Q_meas_init = {1.0f, 0.0f, 0.0f, 0.0f}; // IMU初始测量四元数
	// Eigen::Quaternionf Q_desired{0.6003f, -0.6003f, -0.3735f, -0.3739f}; // (w, x, y, z) // 注意Eigen中四元数赋值的顺序，实数w在首；但是实际上它的内部存储顺序是[x y z w]
	Eigen::Quaternionf Q_desired{0.7070f, 0.0f, -0.7070f, 0.0f}; // (w, x, y, z) // 注意Eigen中四元数赋值的顺序，实数w在首；但是实际上它的内部存储顺序是[x y z w]
    Eigen::Quaternionf Q_offset{Eigen::Quaternionf::Identity()};

    Motor_PDControl_Struct Zbot1234_RL_PD = {
        .Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 2.0f,
    };
	Motor_PDControl_Struct Zbot5_RL_PD = {
        .Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 2.0f,
    };
	Motor_PDControl_Struct Zbot6_RL_PD = {
        .Feedforward_Torque = 0.0f,
		.Tar_Position = 0.0f,
		.Tar_Velocity = 0.0f,
		.Kp = 20.0f,
		.Kd = 2.0f,
    };
	// Motor_PDControl_Struct Zbot5_RL_PD = {
    //     .Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 25.0f,
	// 	.Kd = 3.0f,
    // };
	// Motor_PDControl_Struct Zbot6_RL_PD = {
    //     .Feedforward_Torque = 0.0f,
	// 	.Tar_Position = 0.0f,
	// 	.Tar_Velocity = 0.0f,
	// 	.Kp = 70.0f,
	// 	.Kd = 5.0f,
    // };

	// ************************************************ 接收线程相关变量和成员 ************************************************ //
	
	std::thread _CAN_RX_device_0_thread;
	void CAN_RX_device_0_thread();

    int can_dev0_rx_count;
	int can_dev0_rx_count_thread;

    // CAN转USB设备-接收数据结构体，每个结构体对应不同接收线程，包含两路can共6(8)个模块的电机（和IMU）数据
	USB2CAN_Dev_Struct DEV0_RX = { std::vector<Module_CAN_Recieve_Struct>(motor_dof) };
	// USB2CAN_Dev_Struct DEV0_RX = { std::vector<Module_CAN_Recieve_Struct>(6) }; // 包含ID:1～6 的模块数据
	// // if constexpr 不能用于成员变量声明。必须始终声明该变量以通过编译检查。
	// // 通过三元运算符控制初始化大小：如果 motor_dof == 12，则分配 6 个空间；否则为 0 ,空vector，几乎不占内存。
	// USB2CAN_Dev_Struct DEV1_RX = { std::vector<Module_CAN_Recieve_Struct>((motor_dof == 12) ? 6 : 0) }; // 包含ID:7～12 的模块数据

	// ************************************************ 发送线程相关变量和成员 ************************************************ //
	
	std::thread _CAN_TX_thread;
	void CAN_TX_thread();

	std::vector<float> motor_angles = init_angles; // 发送到电机的目标角度（rad）

	// ************************************************ 策略线程相关变量和成员 ************************************************ //

	std::thread _strategy_thread;
	void Strategy_thread();

	// std::vector<float> out_last = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f}; // 上一次的策略输出百分比
	std::vector<float> out_last = std::vector<float>(action_space, -1.0f); // TODO 确认是否仍然是 -1.0f 初始值 <<==========================

	std::vector<float> position_output = init_angles; // 策略计算的输出--电机绝对位置（rad）

	// ************************************************ 键盘交互线程相关变量和成员 ************************************************ //
	std::thread _keyborad_input;
	void keyborad_input();

	// float command_1 = 1.5f; // laydown joint speed limit
	// float command_1 = 1.0f; // walking joint speed limit
	float command_1 = 0.0f; // velocity x
    float target_heading_yaw = 0.0f; // 目标航向角
	bool print_info_flag = false;


	// ************************************************ 日志线程相关变量和成员 ************************************************ //

	std::thread _log_thread;
	void Log_thread();

	struct LogFrame {
		float timestamp = 0.0;
		float imu_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // (w,x,y,z)
		float motor_pos[motor_dof] = {0};
		float motor_vel[motor_dof] = {0};
		float command1 = 0.0f;
		float heading_err = 0.0f;
		float target_heading_yaw = 0.0f;
	};

	std::queue<LogFrame> log_strategy_queue;
	// std::queue<LogFrame> log_tx_queue;
	// std::queue<LogFrame> log_rx_queue;
	// std::queue<LogFrame> log_rx_2_queue;

	// ************************************************ 线程锁 ************************************************ //

	std::mutex mutex_DEV0_RX;
	std::mutex mutex_keyboard_input;
	std::mutex mutex_position_output; 
	std::mutex mutex_log_strategy_queue;
	// std::mutex mutex_log_tx_queue;
	// std::mutex mutex_log_rx_queue;
	// std::mutex mutex_log_rx_2_queue;

};

#endif
