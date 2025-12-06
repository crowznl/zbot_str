// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#include "Tangair_usb2can.h"

time_t start_tp = 0;

// 主函数循环
void Tangair_usb2can::Spin()
{
    while (all_thread_done_ != true)
    {
        sleep(1); // 延时1s
    }
    printf("~ ALL Exit ~\n");
}

/// @brief 构造函数，初始化
/// @return
Tangair_usb2can::Tangair_usb2can() 
{

    running_ = true;
    all_thread_done_ = false;
    motor_zero_set_already = true; // 注意是否进行过零点设置
    Motor_Ctrl_Mode = PD_MODE; // 选择电机控制模式

    USB2CAN0_ = openUSBCAN("/dev/USB2CAN0");
    if (USB2CAN0_ == -1)
        std::cout << std::endl
                  << "USB2CAN0 open INcorrect!!!" << std::endl;
    else
        std::cout << std::endl
                  << "USB2CAN0 opened ,num=" << USB2CAN0_ << std::endl;

    // 启动成功
    std::cout << std::endl
              << "USB2CAN   NODE INIT__OK   by TANGAIR" << std::endl
              << std::endl
              << std::endl;

    // ********************************************************************** 初 始 化 ********************************************************************** //
    {
        if (motor_zero_set_already == false) // 电机未进行过零点设置 // 若未进行零点设置，切记将TX线程的运动控制代码注释
        {
            Motor_Zero_Set_ALL(Delay_1000us);
            sleep(1);
            ENABLE_ALL_Motor(Delay_1000us);
            sleep(1);
            Read_Clear(12); // 清理接收缓存
        }
        else if (motor_zero_set_already == true) // 电机进行过零点设置
        {
            if (Motor_Ctrl_Mode == PP_MODE)
            {
                ALL_Motor_PP_Init(init_angles); // 使能电机，切换到PP模式，运动到指定角度
                Read_Clear(30); // 清理接收缓存
            }

            else if (Motor_Ctrl_Mode == PD_MODE)
            {
                ALL_Motor_PD_Init(init_angles);
                Read_Clear(12); // 清理接收缓存
            }
        }

        IMU_Set_ALL_SYNC_Mode(Delay_1000us); // 设置IMU为SYNC模式
        sleep(0.5);

        for (size_t i = 0; i < 5; i++) // 采集多次初始IMU四元数数据
        {
            IMU_Send_SYNC(Delay_0us);
            IMU_Get_init_quat();
        }
        IMU_Get_offset_quat(); // 计算校正四元数
        sleep(1);
    }
    // ********************************************************************** 初 始 化 ********************************************************************** //

    // 获取当前系统时间戳
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMill =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    start_tp = tpMill.time_since_epoch().count();
  
    // 创建CAN接收线程，设备1
    _CAN_RX_device_0_thread = std::thread(&Tangair_usb2can::CAN_RX_device_0_thread, this);

    // CAN发送线程
    _CAN_TX_thread = std::thread(&Tangair_usb2can::CAN_TX_thread, this);

    // 键盘输入线程
    _keyborad_input = std::thread(&Tangair_usb2can::keyborad_input, this);
    
    // 策略线程（50Hz）
    _strategy_thread = std::thread(&Tangair_usb2can::Strategy_thread, this);

    // 记录线程（500Hz）
    _log_thread = std::thread(&Tangair_usb2can::Log_thread, this);

#ifdef USE_LIBTORCH
    // 尝试加载模型（可选），路径可在运行时替换
    try {
        policy_model = std::make_shared<torch::jit::script::Module>(torch::jit::load("/home/rain/libtorch/export_model/policy_walking.pt"));
        model_loaded = true;
        std::cout << "Policy model loaded." << std::endl;
    } catch (const std::exception &e) {
        model_loaded = false;
        std::cout << "Warning: failed to load policy model: " << e.what() << std::endl;
    }
#endif
}

/// @brief 析构函数
Tangair_usb2can::~Tangair_usb2can()
{

    running_ = false;

    /*注销线程*/

    // can接收设备0
    _CAN_RX_device_0_thread.join();

    //can发送测试线程
    _CAN_TX_thread.join();
    //键盘输入线程
    _keyborad_input.join();
    // 策略线程停止并join
    if (_strategy_thread.joinable())
        _strategy_thread.join();

    _log_thread.join();
    // 失能电机
    DISABLE_ALL_Motor(100);

    // 关闭设备
    closeUSBCAN(USB2CAN0_);

    all_thread_done_ = true;
}

/// @brief can设备0，接收线程函数
void Tangair_usb2can::CAN_RX_device_0_thread()
{
    can_dev0_rx_count = 0;
    can_dev0_rx_count_thread=0;
    
    while (running_)
    {

        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};
        LogFrame log_rx_data;

        can_dev0_rx_count_thread++;

        // 有数据不会阻塞，若无数据则等待1s
        int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e6);

        // 接收到数据
        if (recieve_re != -1)
        {
            can_dev0_rx_count++;

            if (info_rx.frameType == STANDARD) // 读取IMU数据
            {
                TEMP_ID = ((info_rx.canID) & 0xfff) - 0x480;
                uint8_t index = TEMP_ID - 0x21;  // 将ID映射到0-2的索引,DEV0_RX包含ID21-ID23的IMU数据

                {
                    std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
                    // 收到的数据低字节在前
                    DEV0_RX.Module_CAN_Recieve[index].quat_w = static_cast<float>(static_cast<int16_t>((data_rx[1] << 8) | data_rx[0])) / 10000.0f;
                    DEV0_RX.Module_CAN_Recieve[index].quat_x = static_cast<float>(static_cast<int16_t>((data_rx[3] << 8) | data_rx[2])) / 10000.0f;
                    DEV0_RX.Module_CAN_Recieve[index].quat_y = static_cast<float>(static_cast<int16_t>((data_rx[5] << 8) | data_rx[4])) / 10000.0f;
                    DEV0_RX.Module_CAN_Recieve[index].quat_z = static_cast<float>(static_cast<int16_t>((data_rx[7] << 8) | data_rx[6])) / 10000.0f;
                }
            }

            else if (info_rx.frameType == EXTENDED) // 读取电机数据
            {
                TEMP_ID = (info_rx.canID >> 8) & 0xff;
                uint8_t index = TEMP_ID - 0x01;  // 将ID映射到0-2的索引,DEV0_RX包含ID01-ID03的电机数据

                {
                    std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
                    // 解码
                    DEV0_RX.Module_CAN_Recieve[index].master_id = (info_rx.canID) & 0xff;
                    DEV0_RX.Module_CAN_Recieve[index].motor_id = (info_rx.canID >> 8) & 0xff;
                    DEV0_RX.Module_CAN_Recieve[index].fault_message = (info_rx.canID >> 16) & 0x3f;
                    DEV0_RX.Module_CAN_Recieve[index].motor_state = (info_rx.canID >> 22) & 0x03;
                    DEV0_RX.Module_CAN_Recieve[index].mode = (info_rx.canID >> 24) & 0x1f;

                    if (DEV0_RX.Module_CAN_Recieve[index].mode == 0x02)
                    {
                        // 收到的数据高字节在前
                        DEV0_RX.Module_CAN_Recieve[index].current_position = (data_rx[0] << 8) | (data_rx[1]);
                        DEV0_RX.Module_CAN_Recieve[index].current_speed = (data_rx[2] << 8) | (data_rx[3]);
                        DEV0_RX.Module_CAN_Recieve[index].current_torque = (data_rx[4] << 8) | (data_rx[5]);
                        DEV0_RX.Module_CAN_Recieve[index].current_temp = (data_rx[6] << 8) | (data_rx[7]);

                        // 转换
                        DEV0_RX.Module_CAN_Recieve[index].current_position_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].current_position, (P_MIN), (P_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV0_RX.Module_CAN_Recieve[index].current_speed_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].current_speed, (V_MIN), (V_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV0_RX.Module_CAN_Recieve[index].current_torque_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].current_torque, (T_MIN), (T_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV0_RX.Module_CAN_Recieve[index].current_temp_f = (float)DEV0_RX.Module_CAN_Recieve[index].current_temp / 10;
                    }
                }
            }

            float logtime_now = getTimestamp();

            log_rx_data.timestamp = logtime_now;
            {
                std::lock_guard<std::mutex> lock(mutex_DEV0_RX);

                log_rx_data.motor_pos[0] = DEV0_RX.Module_CAN_Recieve[0].current_position_f;
                log_rx_data.motor_pos[1] = DEV0_RX.Module_CAN_Recieve[1].current_position_f;
                log_rx_data.motor_pos[2] = DEV0_RX.Module_CAN_Recieve[2].current_position_f;
                log_rx_data.motor_pos[3] = DEV0_RX.Module_CAN_Recieve[3].current_position_f;
                log_rx_data.motor_pos[4] = DEV0_RX.Module_CAN_Recieve[4].current_position_f;
                log_rx_data.motor_pos[5] = DEV0_RX.Module_CAN_Recieve[5].current_position_f;

                log_rx_data.motor_vel[0] = DEV0_RX.Module_CAN_Recieve[0].current_speed_f;
                log_rx_data.motor_vel[1] = DEV0_RX.Module_CAN_Recieve[1].current_speed_f;
                log_rx_data.motor_vel[2] = DEV0_RX.Module_CAN_Recieve[2].current_speed_f;
                log_rx_data.motor_vel[3] = DEV0_RX.Module_CAN_Recieve[3].current_speed_f;
                log_rx_data.motor_vel[4] = DEV0_RX.Module_CAN_Recieve[4].current_speed_f;
                log_rx_data.motor_vel[5] = DEV0_RX.Module_CAN_Recieve[5].current_speed_f;
            }

            {
                std::lock_guard<std::mutex> lock(mutex_log_rx_queue);
                log_rx_queue.push(log_rx_data);
            }
        }
    }
    std::cout << "CAN_RX_device_0_thread  Exit~~" << std::endl;
}

// can发送线程函数
void Tangair_usb2can::CAN_TX_thread()
{
    //发送计数
    uint32_t tx_count = 0;
    LogFrame log_tx_data;

    while (running_)
    {
        // CAN发送计数
        tx_count++;

        IMU_Send_SYNC(Delay_300us);

        //使用策略线程计算得到的输出
        {
            std::lock_guard<std::mutex> lock(mutex_position_output); //线程锁
            motor_angles[0] = position_output[0];
            motor_angles[1] = position_output[1];
            motor_angles[2] = position_output[2];
            motor_angles[3] = position_output[3];
            motor_angles[4] = position_output[4];
            motor_angles[5] = position_output[5];
        }

        for (size_t i = 0; i < motor_angles.size(); ++i) {
            motor_angles[i] = std::clamp(motor_angles[i], lower_limit[i], upper_limit[i]);
        }

        if (Motor_Ctrl_Mode == PD_MODE)
        {
            ALL_Motor_PD_Control(Delay_300us, motor_angles);
            // ALL_Motor_PD_Control(Delay_300us, init_angles);
        }
        else if (Motor_Ctrl_Mode == PP_MODE)
        {
            ALL_Motor_PP_Angle_Set(Delay_300us, motor_angles);
            // ALL_Motor_PP_Angle_Set(Delay_300us, init_angles);
        }
    
        float time_now = getTimestamp();

        // 打印数据
        if (tx_count % 1000 == 0)
        {
            {
                std::lock_guard<std::mutex> lock(mutex_command_input);
                std::cout << " 键盘输入: " << command_input << std::endl;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_position_output);
                std::cout << " 网络输出: " << position_output[0] << ", " << position_output[1] << ", " << position_output[2] << ", " 
                                      << position_output[3] << ", " << position_output[4] << ", " << position_output[5] << std::endl;
            }

            std::cout << " 输入电机1的角度: " << motor_angles[0] << std::endl;
            std::cout << " 输入电机2的角度: " << motor_angles[1] << std::endl;
            std::cout << " 输入电机3的角度: " << motor_angles[2] << std::endl;
            std::cout << " 输入电机4的角度: " << motor_angles[3] << std::endl;
            std::cout << " 输入电机5的角度: " << motor_angles[4] << std::endl;
            std::cout << " 输入电机6的角度: " << motor_angles[5] << std::endl;

            {
                std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
                std::cout << " 电机1当前角度: " << DEV0_RX.Module_CAN_Recieve[0].current_position_f << std::endl;
                std::cout << " 电机2当前角度: " << DEV0_RX.Module_CAN_Recieve[1].current_position_f << std::endl;
                std::cout << " 电机3当前角度: " << DEV0_RX.Module_CAN_Recieve[2].current_position_f << std::endl;
                std::cout << " 电机4当前角度: " << DEV0_RX.Module_CAN_Recieve[3].current_position_f << std::endl;
                std::cout << " 电机5当前角度: " << DEV0_RX.Module_CAN_Recieve[4].current_position_f << std::endl;
                std::cout << " 电机6当前角度: " << DEV0_RX.Module_CAN_Recieve[5].current_position_f << std::endl;
            }

            std::cout << " 发送次数:               " << tx_count << std::endl
                      << " CAN转USB设备0 接收次数: " << can_dev0_rx_count << std::endl
                      << " TIME:                  " << time_now << "s" << std::endl
                      << " ***************************************************************** " << std::endl;
        }
    }

    //程序终止时的提示信息
    std::cout << "CAN_TX_test_thread  Exit~~" << std::endl;
    std::cout << std::endl
              << "----------------请输入任意数字，按回车，以结束键盘进程  ----------------------" << std::endl
              << std::endl;
}

// 键盘输入线程
void Tangair_usb2can::keyborad_input()
{
    while (running_)
    {
        std::cin >> command_input; // 键盘输入电机速度限制 // 此处不加线程锁，没有必要
    }
    std::cout << "keyborad_input_thread  Exit~~" << std::endl;
}

// 策略线程：以50Hz运行，读取 command_input 与 zbot 状态，计算 position_output
void Tangair_usb2can::Strategy_thread()
{
    const std::chrono::milliseconds period(20); // 50Hz
    LogFrame log_strategy_data;

    csv_data = loadCSV_invec("/home/rain/Zbot/Project/Proj1205_test_libtorch/csv_plot_py/obs_env0.csv");
    csv_index = 0;

    while (running_)
    {
        auto t0 = std::chrono::steady_clock::now();

        // 读取必要数据并计算输出
        float input_copy;
        std::vector<float> cur_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<float> cur_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        float cur_quat_w = Q_desired.w();
        float cur_quat_x = Q_desired.x();
        float cur_quat_y = Q_desired.y();
        float cur_quat_z = Q_desired.z();

        {
            {
                std::lock_guard<std::mutex> lock(mutex_command_input); 
                input_copy = command_input;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
                IMU_quat_correct();

                cur_quat_w = DEV0_RX.Module_CAN_Recieve[0].quat_w;
                cur_quat_x = DEV0_RX.Module_CAN_Recieve[0].quat_x;
                cur_quat_y = DEV0_RX.Module_CAN_Recieve[0].quat_y;
                cur_quat_z = DEV0_RX.Module_CAN_Recieve[0].quat_z;

                std::cout << " IMU四元数: [" << DEV0_RX.Module_CAN_Recieve[0].quat_w << "," << DEV0_RX.Module_CAN_Recieve[0].quat_x << "," << 
                                               DEV0_RX.Module_CAN_Recieve[0].quat_y << "," << DEV0_RX.Module_CAN_Recieve[0].quat_z << "]" << std::endl;

                cur_pos[0] = DEV0_RX.Module_CAN_Recieve[0].current_position_f;
                cur_pos[1] = DEV0_RX.Module_CAN_Recieve[1].current_position_f;
                cur_pos[2] = DEV0_RX.Module_CAN_Recieve[2].current_position_f;
                cur_pos[3] = DEV0_RX.Module_CAN_Recieve[3].current_position_f;
                cur_pos[4] = DEV0_RX.Module_CAN_Recieve[4].current_position_f;
                cur_pos[5] = DEV0_RX.Module_CAN_Recieve[5].current_position_f;

                cur_vel[0] = DEV0_RX.Module_CAN_Recieve[0].current_speed_f;
                cur_vel[1] = DEV0_RX.Module_CAN_Recieve[1].current_speed_f;
                cur_vel[2] = DEV0_RX.Module_CAN_Recieve[2].current_speed_f;
                cur_vel[3] = DEV0_RX.Module_CAN_Recieve[3].current_speed_f;
                cur_vel[4] = DEV0_RX.Module_CAN_Recieve[4].current_speed_f;
                cur_vel[5] = DEV0_RX.Module_CAN_Recieve[5].current_speed_f;
            }
        }

#ifdef USE_LIBTORCH
        if (model_loaded && policy_model)
        {
            try {
                // // 构造输入tensor：模型接受输入
                // std::vector<float> invec = {cur_quat_w, cur_quat_x, cur_quat_y, cur_quat_z, 
                //                             cur_pos[0] - init_angles[0], cur_pos[1] - init_angles[1], cur_pos[2] - init_angles[2], cur_pos[3] - init_angles[3], cur_pos[4] - init_angles[4], cur_pos[5] - init_angles[5],
                //                             cur_vel[0], cur_vel[1], cur_vel[2], cur_vel[3], cur_vel[4], cur_vel[5],
                //                             out_last[0], out_last[1], out_last[2], out_last[3], out_last[4], out_last[5],
                //                             input_copy
                //                            };
                // ---- 读取CSV作为策略输入 ----
                std::vector<float> invec;
                if (csv_index < csv_data.size()) {
                    invec = csv_data[csv_index++];
                } else {
                    // 播放完就停在最后一行
                    invec = csv_data.back();
                }

                at::Tensor input_tensor = torch::from_blob(invec.data(), {1, (long)invec.size()}).clone();
                // 前向推理
                std::vector<torch::jit::IValue> inputs;
                inputs.push_back(input_tensor);
                at::Tensor out = policy_model->forward(inputs).toTensor().squeeze().tanh();

                out_last.assign(out.data_ptr<float>(), 
                                out.data_ptr<float>() + out.numel());
                relative_tensor += out * input_copy * 0.02f * PI;
                relative_tensor = relative_tensor.clip(-1.0f * PI, 1.0f * PI);
                // at::Tensor类型没有toVector
                std::vector<float> relative_vec(relative_tensor.data_ptr<float>(), 
                                              relative_tensor.data_ptr<float>() + relative_tensor.numel());
                {
                    std::lock_guard<std::mutex> lock(mutex_position_output);
                    for (size_t i = 0; i < relative_vec.size(); ++i) {
                        position_output[i] = relative_vec[i] + init_angles[i];
                    }

                    float time_now = getTimestamp();
                    log_strategy_data.timestamp = time_now;
                    for (size_t i = 0; i < 6; ++i) {
                        log_strategy_data.motor_pos[i] = cur_pos[i] - init_angles[i];
                        log_strategy_data.motor_vel[i] = cur_vel[i];
                    }
                    log_strategy_data.imu_quat[0] = cur_quat_w;
                    log_strategy_data.imu_quat[1] = cur_quat_x;
                    log_strategy_data.imu_quat[2] = cur_quat_y;
                    log_strategy_data.imu_quat[3] = cur_quat_z;
                    {
                        std::lock_guard<std::mutex> lock(mutex_log_strategy_queue);
                        log_strategy_queue.push(log_strategy_data);
                    }
                }
                
            } catch (const std::exception &e) {
                // 推理失败则回退到简单策略
                std::lock_guard<std::mutex> lock(mutex_position_output);
                position_output = init_angles;
                std::cout << "Policy inference failed: " << e.what() << std::endl;
            }
        }
        else
#endif
        {
            // 简单回退策略
            std::lock_guard<std::mutex> lock(mutex_position_output);
            position_output = init_angles;
        }

        // 固定周期等待
        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < period)
            std::this_thread::sleep_for(period - elapsed);
    }
    std::cout << "Strategy_thread Exit~~" << std::endl;
}

// 日志线程
void Tangair_usb2can::Log_thread() {
    // ================== 在这里修改存储路径 ==================
    std::string log_path = "/home/rain/Zbot/Project/Proj1205_test_libtorch/csv_plot_py/";
    
    // 确保路径以斜杠结尾
    if (!log_path.empty() && log_path.back() != '/') {
        log_path += '/';
    }
    
    using namespace std::chrono;
    auto start_time = steady_clock::now();
    int file_index = 0;
    std::ofstream rx_file, strategy_file;

    auto openNewFiles = [&](int index) {
        if (rx_file.is_open()) rx_file.close();
        if (strategy_file.is_open()) strategy_file.close();

        // 使用 log_path 变量
        std::string rx_filename = log_path + "log_rx_" + std::to_string(index) + ".csv";
        rx_file.open(rx_filename, std::ios::out);
        if (!rx_file.is_open()) {
            std::cerr << "无法打开文件: " << rx_filename << std::endl;
            return;
        }
        // 修改表头：四元数在前
        rx_file << "timestamp,imu_w,imu_x,imu_y,imu_z,pos1,pos2,pos3,pos4,pos5,pos6,vel1,vel2,vel3,vel4,vel5,vel6\n";

        std::string strategy_filename = log_path + "log_strategy_" + std::to_string(index) + ".csv";
        strategy_file.open(strategy_filename, std::ios::out);
        if (!strategy_file.is_open()) {
            std::cerr << "无法打开文件: " << strategy_filename << std::endl;
            return;
        }
        // 修改表头：四元数在前
        strategy_file << "timestamp,imu_w,imu_x,imu_y,imu_z,pos1,pos2,pos3,pos4,pos5,pos6,vel1,vel2,vel3,vel4,vel5,vel6\n";
        
        std::cout << "创建日志文件: " << rx_filename << " 和 " << strategy_filename << std::endl;
    };

    openNewFiles(file_index);

    while (running_) {
        auto now = steady_clock::now();
        if (duration_cast<seconds>(now - start_time).count() >= 100) {
            file_index++;
            openNewFiles(file_index);
            start_time = now;
        }

        // 批量收集 RX 队列数据
        std::vector<LogFrame> rx_batch;
        {
            std::lock_guard<std::mutex> lock(mutex_log_rx_queue);
            while (!log_rx_queue.empty()) {
                rx_batch.push_back(log_rx_queue.front());
                log_rx_queue.pop();
            }
        }

        // 批量收集策略队列数据
        std::vector<LogFrame> strategy_batch;
        {
            std::lock_guard<std::mutex> lock(mutex_log_strategy_queue);
            while (!log_strategy_queue.empty()) {
                strategy_batch.push_back(log_strategy_queue.front());
                log_strategy_queue.pop();
            }
        }

        // 批量写入 RX 日志
        if (!rx_batch.empty() && rx_file.is_open()) {
            for (auto &entry : rx_batch) {
                rx_file << std::fixed << std::setprecision(6) << entry.timestamp << ",";
                
                // 先写入 IMU 四元数 (w,x,y,z)
                for (int i = 0; i < 4; ++i) {
                    rx_file << entry.imu_quat[i] << ",";
                }
                
                // 再写入电机位置
                for (int i = 0; i < 6; ++i) {
                    rx_file << entry.motor_pos[i] << ",";
                }
                
                // 最后写入电机速度（最后一个不加逗号）
                for (int i = 0; i < 6; ++i) {
                    rx_file << entry.motor_vel[i];
                    if (i < 5) {
                        rx_file << ",";
                    } else {
                        rx_file << "\n";
                    }
                }
            }
            rx_file.flush();  // 确保数据写入磁盘
        }

        // 批量写入策略日志
        if (!strategy_batch.empty() && strategy_file.is_open()) {
            for (auto &entry : strategy_batch) {
                strategy_file << std::fixed << std::setprecision(6) << entry.timestamp << ",";
                
                // 先写入 IMU 四元数 (w,x,y,z)
                for (int i = 0; i < 4; ++i) {
                    strategy_file << entry.imu_quat[i] << ",";
                }
                
                // 再写入电机位置
                for (int i = 0; i < 6; ++i) {
                    strategy_file << entry.motor_pos[i] << ",";
                }
                
                // 最后写入电机速度（最后一个不加逗号）
                for (int i = 0; i < 6; ++i) {
                    strategy_file << entry.motor_vel[i];
                    if (i < 5) {
                        strategy_file << ",";
                    } else {
                        strategy_file << "\n";
                    }
                }
            }
            strategy_file.flush();  // 确保数据写入磁盘
        }

        // 队列为空则短暂休眠
        if (rx_batch.empty() && strategy_batch.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    if (rx_file.is_open()) rx_file.close();
    if (strategy_file.is_open()) strategy_file.close();
    
    std::cout << "日志线程结束，文件保存路径: " << log_path << std::endl;
}

/*****************************************************************************************************/
/*********************************       ***电机相关***      ***********************************************/
/*****************************************************************************************************/


/// @brief 电机使能
/// @param dev
/// @param channel
/// @param motor_id
void Tangair_usb2can::Motor_Enable(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x03;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));

    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

/// @brief 电机失能
/// @param dev
/// @param channel
/// @param motor_id
void Tangair_usb2can::Motor_Disable(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x04;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    // Data_CAN[0] = 1; // 用于清除故障位
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

void Tangair_usb2can::ENABLE_ALL_Motor(int delay_us)
{
    Motor_Enable(USB2CAN0_, 2, 0x01);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Enable(USB2CAN0_, 1, 0x04);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Enable(USB2CAN0_, 2, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Enable(USB2CAN0_, 1, 0x05);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Enable(USB2CAN0_, 2, 0x03);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Enable(USB2CAN0_, 1, 0x06);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::DISABLE_ALL_Motor(int delay_us)
{   
    Motor_Disable(USB2CAN0_, 2, 0x01);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Disable(USB2CAN0_, 1, 0x04);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Disable(USB2CAN0_, 2, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Disable(USB2CAN0_, 1, 0x05);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Disable(USB2CAN0_, 2, 0x03);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Disable(USB2CAN0_, 1, 0x06);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::Motor_Mode_Change(int32_t dev, uint8_t channel, uint32_t motor_id , uint8_t mode)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }

    Data_CAN[0] = 0x05;
    Data_CAN[1] = 0x70;
    Data_CAN[4] = mode;
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

void Tangair_usb2can::PP_Vel_Max_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float velmax)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    
    Data_CAN[0] = 0x24;
    Data_CAN[1] = 0x70;
    std::memcpy(&Data_CAN[4], &velmax, sizeof(velmax));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

void Tangair_usb2can::PP_Acc_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float acc)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    
    Data_CAN[0] = 0x25;
    Data_CAN[1] = 0x70;
    std::memcpy(&Data_CAN[4], &acc, sizeof(acc));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

void Tangair_usb2can::PP_Mode_Set(int32_t dev, uint8_t channel, uint32_t motor_id, float velmax, float acc, int delay_us)
{
    Motor_Mode_Change(dev, channel, motor_id, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor_Enable(dev, channel, motor_id);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    PP_Vel_Max_Set(dev, channel, motor_id, velmax);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    PP_Acc_Set(dev, channel, motor_id, acc);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Tangair_usb2can::ALL_Motor_PP_Mode_Set(int delay_us)
{
    PP_Mode_Set(USB2CAN0_, 2, 0x01, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Mode_Set(USB2CAN0_, 1, 0x04, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Mode_Set(USB2CAN0_, 2, 0x02, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Mode_Set(USB2CAN0_, 1, 0x05, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Mode_Set(USB2CAN0_, 2, 0x03, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Mode_Set(USB2CAN0_, 1, 0x06, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}

void Tangair_usb2can::PP_Angle_Set(int32_t dev, uint8_t channel, uint32_t motor_id , float angle)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x12;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    Data_CAN[0] = 0x16;
    Data_CAN[1] = 0x70;
    std::memcpy(&Data_CAN[4], &angle, sizeof(angle));
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

void Tangair_usb2can::ALL_Motor_PP_Angle_Set(int delay_us, std::vector<float> motor_angles)
{
    PP_Angle_Set(USB2CAN0_, 2, 0x01, -motor_angles[0]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Angle_Set(USB2CAN0_, 1, 0x04, -motor_angles[3]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Angle_Set(USB2CAN0_, 2, 0x02, -motor_angles[1]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Angle_Set(USB2CAN0_, 1, 0x05, -motor_angles[4]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Angle_Set(USB2CAN0_, 2, 0x03, -motor_angles[2]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    PP_Angle_Set(USB2CAN0_, 1, 0x06, -motor_angles[5]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}

void Tangair_usb2can::ALL_Motor_Mode_PP2PD(int delay_us)
{
    Motor_Mode_Change(USB2CAN0_, 2, 0x01, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor_Mode_Change(USB2CAN0_, 1, 0x04, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor_Mode_Change(USB2CAN0_, 2, 0x02, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor_Mode_Change(USB2CAN0_, 1, 0x05, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor_Mode_Change(USB2CAN0_, 2, 0x03, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor_Mode_Change(USB2CAN0_, 1, 0x06, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}

void Tangair_usb2can::ALL_Motor_PP_Init(std::vector<float> motor_angles)
{
    // 所有电机设置为PP模式
    ALL_Motor_PP_Mode_Set(Delay_10000us);
    sleep(0.1);

    // 以PP模式运动到初始位置
    ALL_Motor_PP_Angle_Set(Delay_10000us, motor_angles);
    sleep(3);
}
void Tangair_usb2can::Motor_Zero_Set(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x06;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }
    
    Data_CAN[0] = 0x01;
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

void Tangair_usb2can::Motor_Zero_Set_ALL(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now();//这一句耗时50us

    Motor_Zero_Set(USB2CAN0_, 2, 0x01);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_Zero_Set(USB2CAN0_, 1, 0x04);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_Zero_Set(USB2CAN0_, 2, 0x02);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_Zero_Set(USB2CAN0_, 1, 0x05);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_Zero_Set(USB2CAN0_, 2, 0x03);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_Zero_Set(USB2CAN0_, 1, 0x06);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Tangair_usb2can::ALL_Motor_PD_Init(std::vector<float> motor_angles)
{
    ENABLE_ALL_Motor(Delay_1000us); // 使能电机
    sleep(1);
    ALL_Motor_PD_Control(Delay_1000us, motor_angles); // PD模式运动到指定角度
    sleep(3);
}

void Tangair_usb2can::Motor_PD_Control(int32_t dev, uint8_t channel, uint32_t motor_id, Motor_PDControl_Struct *Motor_PDControl, float position)
{
    Motor_PDControl->Tar_Position = position;

    ID_CAN.id = motor_id;
    ID_CAN.exdata = float_to_uint(Motor_PDControl->Feedforward_Torque, T_MIN, T_MAX, 16);
    ID_CAN.mode = 0x1;

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));

    Data_CAN[0] = float_to_uint(Motor_PDControl->Tar_Position, P_MIN, P_MAX, 16) >> 8;
    Data_CAN[1] = float_to_uint(Motor_PDControl->Tar_Position, P_MIN, P_MAX, 16);
    Data_CAN[2] = float_to_uint(Motor_PDControl->Tar_Velocity, V_MIN, V_MAX, 16) >> 8;
    Data_CAN[3] = float_to_uint(Motor_PDControl->Tar_Velocity, V_MIN, V_MAX, 16);
    Data_CAN[4] = float_to_uint(Motor_PDControl->Kp, KP_MIN, KP_MAX, 16) >> 8;
    Data_CAN[5] = float_to_uint(Motor_PDControl->Kp, KP_MIN, KP_MAX, 16);
    Data_CAN[6] = float_to_uint(Motor_PDControl->Kd, KD_MIN, KD_MAX, 16) >> 8;
    Data_CAN[7] = float_to_uint(Motor_PDControl->Kd, KD_MIN, KD_MAX, 16);

    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
}

void Tangair_usb2can::ALL_Motor_PD_Control(int delay_us, std::vector<float> motor_angles)
{
    auto t = std::chrono::high_resolution_clock::now();//这一句耗时50us

    Motor_PD_Control(USB2CAN0_, 2, 0x01, &Motor_PDControl, -motor_angles[0]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 1, 0x04, &Motor_PDControl, -motor_angles[3]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 2, 0x02, &Motor_PDControl, -motor_angles[1]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 1, 0x05, &Motor_PDControl, -motor_angles[4]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 2, 0x03, &Motor_PDControl, -motor_angles[2]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 1, 0x06, &Motor_PDControl, -motor_angles[5]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Tangair_usb2can::IMU_Set_SYNC_Mode(int32_t dev, uint8_t channel, uint8_t imu_id)
{
    txMsg_CAN_IMU.canID = 0x600 | (imu_id & 0xFF);
    Data_CAN[0] = 0x2F;
    Data_CAN[1] = 0x03;     // 设置TPDO4，对应四元数输出
    Data_CAN[2] = 0x18;
    Data_CAN[3] = 0x02;
    Data_CAN[4] = 0x01;     // 设置同步模式
    Data_CAN[5] = 0x00;
    Data_CAN[6] = 0x00;
    Data_CAN[7] = 0x00;
    sendUSBCAN(dev, channel, &txMsg_CAN_IMU, Data_CAN);
}

void Tangair_usb2can::IMU_Set_ALL_SYNC_Mode(int delay_us)
{
    auto t = std::chrono::high_resolution_clock::now();

    IMU_Set_SYNC_Mode(USB2CAN0_, 1, 0x21);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Tangair_usb2can::IMU_Send_SYNC(int delay_us)
{
    txMsg_CAN_IMU.canID = 0x80;
    for (uint8_t i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0x00;
    }
    
    auto t = std::chrono::high_resolution_clock::now();

    sendUSBCAN(USB2CAN0_, 1, &txMsg_CAN_IMU, Data_CAN);
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Tangair_usb2can::IMU_Get_init_quat()
{
    uint8_t channel;
    FrameInfo info_rx;
    uint8_t data_rx[8] = {0};
    // 有数据不会阻塞，若无数据则等待1s
    int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e6);

    // 接收到数据
    if (recieve_re != -1)
    {
        if (info_rx.frameType == STANDARD) // 读取IMU数据
        {
            // 收到的数据低字节在前
            Q_meas_init[0] = static_cast<float>(static_cast<int16_t>((data_rx[1] << 8) | data_rx[0])) / 10000.0f;
            Q_meas_init[1] = static_cast<float>(static_cast<int16_t>((data_rx[3] << 8) | data_rx[2])) / 10000.0f;
            Q_meas_init[2] = static_cast<float>(static_cast<int16_t>((data_rx[5] << 8) | data_rx[4])) / 10000.0f;
            Q_meas_init[3] = static_cast<float>(static_cast<int16_t>((data_rx[7] << 8) | data_rx[6])) / 10000.0f;
            std::cout << "Q_meas_init:[" << Q_meas_init[0] << "," << Q_meas_init[1] << "," << Q_meas_init[2] << "," << Q_meas_init[3] << "]" << std::endl;
        }
    }
}

void Tangair_usb2can::IMU_Get_offset_quat()
{
    // 创建Eigen四元数 (注意顺序：w, x, y, z)
    Eigen::Quaternionf q_meas(
        Q_meas_init[0],  // w
        Q_meas_init[1],  // x  
        Q_meas_init[2],  // y
        Q_meas_init[3]   // z
    );
    
    // 归一化
    q_meas.normalize();
    Q_desired.normalize();
    
    // 计算校正四元数: Q_offset = Q_desired * q_meas.conjugate()
    Q_offset = Q_desired * q_meas.conjugate();
    // Q_offset = q_meas.conjugate() * Q_desired;
    Q_offset.normalize();
    
    std::cout << "Calibration Complete!" << std::endl;
    std::cout << "Q_offset:[" << Q_offset.w() << "," << Q_offset.x() << "," 
            << Q_offset.y() << "," << Q_offset.z() << "]" << std::endl;
}

void Tangair_usb2can::IMU_quat_correct()
{
    Eigen::Quaternionf q_current(
        DEV0_RX.Module_CAN_Recieve[0].quat_w,
        DEV0_RX.Module_CAN_Recieve[0].quat_x,
        DEV0_RX.Module_CAN_Recieve[0].quat_y,
        DEV0_RX.Module_CAN_Recieve[0].quat_z
    );
    
    q_current.normalize();
    Eigen::Quaternionf q_corrected = Q_offset * q_current;
    q_corrected.normalize();
    
    // 存回数据
    DEV0_RX.Module_CAN_Recieve[0].quat_w = q_corrected.w();
    DEV0_RX.Module_CAN_Recieve[0].quat_x = q_corrected.x();
    DEV0_RX.Module_CAN_Recieve[0].quat_y = q_corrected.y();
    DEV0_RX.Module_CAN_Recieve[0].quat_z = q_corrected.z();
}

std::vector<std::vector<float>> Tangair_usb2can::loadCSV_invec(const std::string& path) {
    std::vector<std::vector<float>> data;
    std::ifstream file(path);
    std::string line;

    bool is_first_line = true;
    int line_num = 0;

    while (std::getline(file, line)) {
        line_num++;

        // 跳过表头
        if (is_first_line) {
            is_first_line = false;
            continue;
        }

        std::stringstream ss(line);
        std::string cell;

        std::vector<float> row_23;
        int col_index = 0;

        while (std::getline(ss, cell, ',')) {

            // 去掉前后空白和 \r
            cell.erase(0, cell.find_first_not_of(" \t\r"));
            cell.erase(cell.find_last_not_of(" \t\r") + 1);

            // 只处理 2~24 列，即 col_index 1~23
            if (col_index >= 1 && col_index <= 23) {
                if (!cell.empty()) {
                    try {
                        row_23.push_back(std::stof(cell));
                    } catch (...) {
                        std::cout << "Warning: CSV 第 " << line_num
                                  << " 行第 " << (col_index + 1)
                                  << " 列无法转换为数字，值='" << cell
                                  << "'，该行已跳过。\n";
                        row_23.clear();
                        break;
                    }
                } else {
                    row_23.push_back(0.0f);  // 空值给默认0，你可以换成需要的默认值
                }
            }

            col_index++;
        }

        // 必须是 23 列
        if (row_23.size() != 23) {
            std::cout << "Warning: 第 " << line_num
                      << " 行有效列数不是23，已跳过。\n";
            continue;
        }

        data.push_back(row_23);
    }

    return data;
}

void Tangair_usb2can::Read_Clear(int num)
{
    uint8_t channel;
    FrameInfo info_rx;
    uint8_t data_rx[8] = {0};

    for (int i = 0; i < num; i++)
    {
        // 有数据不会阻塞，若无数据则等待0.1s
        readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e5);
    }
}

/// @brief 辅助函数
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max)
        x = x_max;
    else if (x < x_min)
        x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
float getTimestamp() {
    // 获取当前系统时间戳
    std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> tpMill =
        std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
    time_t now_tp = tpMill.time_since_epoch().count(); 
    
    time_t relavitive_tp = now_tp - start_tp;
    float timestamp = relavitive_tp / 1000.0f;

    return timestamp;
}



// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0

