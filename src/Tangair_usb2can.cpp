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
        if (motor_zero_set_already == false) // 电机未进行过零点设置
        {
            Motor_Zero_Set_ALL(Delay_1000us);
            sleep(1);
            ENABLE_ALL_Motor(Delay_1000us);
            sleep(1);
        }
        else if (motor_zero_set_already == true) // 电机进行过零点设置
        {
            Init_Motor_ALL(10000); // 使能电机并运动到指定角度
            sleep(3);
        }

        IMU_Set_ALL_SYNC_Mode(Delay_1000us); // 设置IMU为SYNC模式
        sleep(1);

        for (size_t i = 0; i < 8; i++) // 采集多次初始IMU四元数数据
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

                // 收到的数据低字节在前
                DEV0_RX.Module_CAN_Recieve[index].quat_w = static_cast<float>(static_cast<int16_t>((data_rx[1] << 8) | data_rx[0])) / 10000.0f;
                DEV0_RX.Module_CAN_Recieve[index].quat_x = static_cast<float>(static_cast<int16_t>((data_rx[3] << 8) | data_rx[2])) / 10000.0f;
                DEV0_RX.Module_CAN_Recieve[index].quat_y = static_cast<float>(static_cast<int16_t>((data_rx[5] << 8) | data_rx[4])) / 10000.0f;
                DEV0_RX.Module_CAN_Recieve[index].quat_z = static_cast<float>(static_cast<int16_t>((data_rx[7] << 8) | data_rx[6])) / 10000.0f;
            }

            else if (info_rx.frameType == EXTENDED) // 读取电机数据
            {
                TEMP_ID = (info_rx.canID >> 8) & 0xff;
                uint8_t index = TEMP_ID - 0x01;  // 将ID映射到0-2的索引,DEV0_RX包含ID01-ID03的电机数据

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
                    DEV0_RX.Module_CAN_Recieve[index].current_speed_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].current_speed, (V_MIN), (V_MAX), 16);    // 灵足电机,此处为RS00参数 // 电机顺时针为角度增加，所以加负号
                    DEV0_RX.Module_CAN_Recieve[index].current_torque_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].current_torque, (T_MIN), (T_MAX), 16);  // 灵足电机,此处为RS00参数
                    DEV0_RX.Module_CAN_Recieve[index].current_temp_f = (float)DEV0_RX.Module_CAN_Recieve[index].current_temp / 10;
                }
            }

            float logtime_now = getTimestamp();

            log_rx_data.timestamp = logtime_now;
            // log_rx_data.quat_w = DEV0_RX.Module_CAN_Recieve[0].quat_w;
            // log_rx_data.quat_x = DEV0_RX.Module_CAN_Recieve[0].quat_x;
            // log_rx_data.quat_y = DEV0_RX.Module_CAN_Recieve[0].quat_y;
            // log_rx_data.quat_z = DEV0_RX.Module_CAN_Recieve[0].quat_z;
            log_rx_data.motor_pos[0] = DEV0_RX.Module_CAN_Recieve[0].current_position_f;
            log_rx_data.motor_pos[1] = DEV0_RX.Module_CAN_Recieve[1].current_position_f;
            log_rx_data.motor_pos[2] = DEV0_RX.Module_CAN_Recieve[2].current_position_f;
            log_rx_data.motor_pos[3] = DEV0_RX.Module_CAN_Recieve[3].current_position_f;
            log_rx_data.motor_pos[4] = DEV0_RX.Module_CAN_Recieve[4].current_position_f;
            log_rx_data.motor_pos[5] = DEV0_RX.Module_CAN_Recieve[5].current_position_f;

            std::lock_guard<std::mutex> lock(log_rx_mutex);
            log_rx_queue.push(log_rx_data);
        }
    }
    std::cout << "CAN_RX_device_0_thread  Exit~~" << std::endl;
}

/*****************************************************************************************************/
/*********************************       ***发送相关***      ***********************************************/
/*****************************************************************************************************/
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
            std::lock_guard<std::mutex> lock(strategy_mutex); //线程锁
            motor_angles[0] = position_output[0];
            motor_angles[1] = position_output[1];
            motor_angles[2] = position_output[2];
            motor_angles[3] = position_output[3];
            motor_angles[4] = position_output[4];
            motor_angles[5] = position_output[5];

            for (size_t i = 0; i < motor_angles.size(); ++i) {
                motor_angles[i] = std::clamp(motor_angles[i], lower_limit[i], upper_limit[i]);
            }
        }

        ALL_Motor_PD_Control(Delay_300us, motor_angles);
        // ALL_Motor_PD_Control(Delay_300us, init_angles);
    
        float time_now = getTimestamp();

        log_tx_data.timestamp = time_now;
        log_tx_data.motor_pos[0] = motor_angles[0];
        log_tx_data.motor_pos[1] = motor_angles[1];
        log_tx_data.motor_pos[2] = motor_angles[2];
        log_tx_data.motor_pos[3] = motor_angles[3];
        log_tx_data.motor_pos[4] = motor_angles[4];
        log_tx_data.motor_pos[5] = motor_angles[5];

        std::lock_guard<std::mutex> lock(log_tx_mutex);
        log_tx_queue.push(log_tx_data);

        // 打印数据
        if (tx_count % 1000 == 0)
        {
            std::cout << " 键盘输入: " << command_input << std::endl;
            std::cout << " 网络输出: " << position_output[0] << ", " << position_output[1] << ", " << position_output[2] << ", " 
                                      << position_output[3] << ", " << position_output[4] << ", " << position_output[5] << std::endl;

            std::cout << " 输入电机1的角度: " << motor_angles[0] << std::endl;
            std::cout << " 输入电机2的角度: " << motor_angles[1] << std::endl;
            std::cout << " 输入电机3的角度: " << motor_angles[2] << std::endl;
            std::cout << " 输入电机4的角度: " << motor_angles[3] << std::endl;
            std::cout << " 输入电机5的角度: " << motor_angles[4] << std::endl;
            std::cout << " 输入电机6的角度: " << motor_angles[5] << std::endl;

            std::cout << " 电机1当前角度: " << DEV0_RX.Module_CAN_Recieve[0].current_position_f << std::endl;
            std::cout << " 电机2当前角度: " << DEV0_RX.Module_CAN_Recieve[1].current_position_f << std::endl;
            std::cout << " 电机3当前角度: " << DEV0_RX.Module_CAN_Recieve[2].current_position_f << std::endl;
            std::cout << " 电机4当前角度: " << DEV0_RX.Module_CAN_Recieve[3].current_position_f << std::endl;
            std::cout << " 电机5当前角度: " << DEV0_RX.Module_CAN_Recieve[4].current_position_f << std::endl;
            std::cout << " 电机6当前角度: " << DEV0_RX.Module_CAN_Recieve[5].current_position_f << std::endl;

            std::cout << " IMU四元数: [" << DEV0_RX.Module_CAN_Recieve[0].quat_w << "," << DEV0_RX.Module_CAN_Recieve[0].quat_x << "," << 
                                           DEV0_RX.Module_CAN_Recieve[0].quat_y << "," << DEV0_RX.Module_CAN_Recieve[0].quat_z << "]" << std::endl;
            // 由于IMU的ID为0x21，所以四元数存在EV0_RX.Module_CAN_Recieve[0]内，与电机ID为0x01的电机数据存在同一个结构体内，实际上IMU实际放置位置并不与模块1绑定

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

// 键盘输入线程，可以键盘修改电机速度
void Tangair_usb2can::keyborad_input()
{

    while (running_)
    {
        std::cin >> command_input; // 键盘输入电机速度限制
    }
    std::cout << "keyborad_input_thread  Exit~~" << std::endl;
}

// 策略线程：以50Hz运行，读取 command_input 与 zbot 状态，计算 position_output
void Tangair_usb2can::Strategy_thread()
{
    const std::chrono::milliseconds period(20); // 50Hz
    LogFrame log_strategy_data;
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
            std::lock_guard<std::mutex> lock(strategy_mutex); //线程锁
            input_copy = command_input;

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

#ifdef USE_LIBTORCH
        if (model_loaded && policy_model)
        {
            try {
                // 构造输入tensor：模型接受输入
                std::vector<float> invec = {cur_quat_w, cur_quat_x, cur_quat_y, cur_quat_z, 
                                            cur_pos[0] - init_angles[0], cur_pos[1] - init_angles[1], cur_pos[2] - init_angles[2], cur_pos[3] - init_angles[3], cur_pos[4] - init_angles[4], cur_pos[5] - init_angles[5],
                                            cur_vel[0], cur_vel[1], cur_vel[2], cur_vel[3], cur_vel[4], cur_vel[5],
                                            out_last[0], out_last[1], out_last[2], out_last[3], out_last[4], out_last[5],
                                            input_copy
                                           };

                at::Tensor input_tensor = torch::from_blob(invec.data(), {1, (long)invec.size()}).clone();
                // 前向推理
                std::vector<torch::jit::IValue> inputs;
                inputs.push_back(input_tensor);
                at::Tensor out = policy_model->forward(inputs).toTensor().squeeze().tanh();
                out_last.assign(out.data_ptr<float>(), 
                                out.data_ptr<float>() + out.numel());
                // std::cout << "out shape: " << out.sizes() << std::endl;
                relative_tensor += out * input_copy * 0.02f;
                relative_tensor = relative_tensor.clip(-1.0f * PI, 1.0f * PI);
                
                {
                    float* temp = relative_tensor.data_ptr<float>();
                    std::lock_guard<std::mutex> lock(log_strategy_mutex);
                    float time_now = getTimestamp();
                    log_strategy_data.timestamp = time_now;
                    for(int i=0; i<6; ++i) 
                    {
                        log_strategy_data.motor_pos[i] = temp[i];
                    }
                    log_strategy_queue.push(log_strategy_data);
                }
                // at::Tensor类型没有toVector
                std::vector<float> relative_vec(relative_tensor.data_ptr<float>(), 
                                              relative_tensor.data_ptr<float>() + relative_tensor.numel());
                
                std::lock_guard<std::mutex> lock(strategy_mutex);
                for (size_t i = 0; i < relative_vec.size(); ++i) {
                    position_output[i] = relative_vec[i] + init_angles[i];
                }
                
            } catch (const std::exception &e) {
                // 推理失败则回退到简单策略
                std::lock_guard<std::mutex> lock(strategy_mutex);
                position_output = init_angles;
                std::cout << "Policy inference failed: " << e.what() << std::endl;
            }
        }
        else
#endif
        {
            // 简单回退策略
            std::lock_guard<std::mutex> lock(strategy_mutex);
            position_output = init_angles;
        }

        // 固定周期等待
        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < period)
            std::this_thread::sleep_for(period - elapsed);
    }
    std::cout << "Strategy_thread Exit~~" << std::endl;
}

// 记录线程
void Tangair_usb2can::Log_thread() {
    using namespace std::chrono;
    auto start_time = steady_clock::now();
    int file_index = 0;
    std::ofstream tx_file, strategy_file;

    // 打开新日志文件的Lambda
    auto openNewFiles = [&](int index) {
        // 关闭旧文件
        if (tx_file.is_open()) tx_file.close();
        if (strategy_file.is_open()) strategy_file.close();
        
        // 打开发送数据文件
        std::string tx_filename = "log_tx_" + std::to_string(index) + ".csv";
        tx_file.open(tx_filename, std::ios::out);
        tx_file << "timestamp,motor1,motor2,motor3,motor4,motor5,motor6,qw,qx,qy,qz\n";
        std::cout << "[Logger] New TX file: " << tx_filename << std::endl;
        
        // 打开策略数据文件
        std::string strategy_filename = "log_strategy_" + std::to_string(index) + ".csv";
        strategy_file.open(strategy_filename, std::ios::out);
        strategy_file << "timestamp,motor1,motor2,motor3,motor4,motor5,motor6,qw,qx,qy,qz\n";
        std::cout << "[Logger] New Strategy file: " << strategy_filename << std::endl;
    };

    openNewFiles(file_index);

    while (running_) {
        // 每100秒轮换日志文件
        auto now = steady_clock::now();
        if (duration_cast<seconds>(now - start_time).count() >= 100) {
            file_index++;
            openNewFiles(file_index);
            start_time = now;
        }

        bool hasTxData = false;
        bool hasStrategyData = false;
        LogFrame log_tx_data;
        LogFrame log_strategy_data;

        {   // 处理发送数据队列
            std::lock_guard<std::mutex> lock(log_tx_mutex);
            if (!log_tx_queue.empty()) {
                log_tx_data = log_tx_queue.front();
                log_tx_queue.pop();
                hasTxData = true;
            }
        }

        {   // 处理策略数据队列
            std::lock_guard<std::mutex> lock(log_strategy_mutex);
            if (!log_strategy_queue.empty()) {
                log_strategy_data = log_strategy_queue.front();
                log_strategy_queue.pop();
                hasStrategyData = true;
            }
        }

        // 写入发送数据到文件
        if (hasTxData && tx_file.is_open()) {
            tx_file << std::fixed << std::setprecision(6)
                    << log_tx_data.timestamp << ",";
            for (int i = 0; i < 6; ++i) tx_file << log_tx_data.motor_pos[i] << ",";
            tx_file << log_tx_data.quat_w << "," << log_tx_data.quat_x << ","
                    << log_tx_data.quat_y << "," << log_tx_data.quat_z << "\n";
        }

        // 写入策略数据到文件
        if (hasStrategyData && strategy_file.is_open()) {
            strategy_file << std::fixed << std::setprecision(6)
                         << log_strategy_data.timestamp << ",";
            for (int i = 0; i < 6; ++i) strategy_file << log_strategy_data.motor_pos[i] << ",";
            strategy_file << log_strategy_data.quat_w << "," << log_strategy_data.quat_x << ","
                         << log_strategy_data.quat_y << "," << log_strategy_data.quat_z << "\n";
        }

        // 如果两个队列都为空，则短暂休眠
        if (!hasTxData && !hasStrategyData) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // 关闭文件
    if (tx_file.is_open()) tx_file.close();
    if (strategy_file.is_open()) strategy_file.close();
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

void Tangair_usb2can::Motor_Initpos_Get(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    ID_CAN.id = motor_id;
    ID_CAN.exdata = 0xfd; // 主机ID
    ID_CAN.mode = 0x11;

    FrameInfo info_rx;
    uint8_t data_rx[8] = {0};

    txMsg_CAN_Motor.canID = ((ID_CAN.id & 0xff) | ((ID_CAN.exdata & 0xffff) << 8) | ((ID_CAN.mode & 0x1f) << 24));
    for (int i = 0; i < 8; i++)
    {
        Data_CAN[i] = 0;
    }

    Data_CAN[0] = 0x16;
    Data_CAN[1] = 0x70; 
    sendUSBCAN(dev, channel, &txMsg_CAN_Motor, Data_CAN);
    sleep(0.1);

    // 有数据不会阻塞，若无数据则等待1s
    int recieve_re = readUSBCAN(dev, &channel, &info_rx, data_rx, 1e6);
    if (recieve_re != -1)
    {
        TEMP_ID = (info_rx.canID >> 8) & 0xff;
        uint8_t index = TEMP_ID - 0x01;  // 将ID映射到0-2的索引,DEV0_RX包含ID01-ID03的电机数据

        int32_t temp_pos = (data_rx[7] << 24) | (data_rx[6] << 16) | (data_rx[5] << 8) | (data_rx[4]);
        std::memcpy(&DEV0_RX.Module_CAN_Recieve[index].motor_init_position, &temp_pos, sizeof(float));
        std::cout << "Get InitPos Successful :" << DEV0_RX.Module_CAN_Recieve[index].motor_init_position << std::endl;
        printf("data:");
        for (int i = 0; i < 8; i++) {
            printf("%02x ", data_rx[i]);
        }
        printf("\n");
    }
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

// 暂时不用
void Tangair_usb2can::Init_Motor(int32_t dev, uint8_t channel, uint32_t motor_id)
{
    Motor_Enable(dev, channel, motor_id);
}

void Tangair_usb2can::Init_Motor_ALL(int delay_us)
{
    ENABLE_ALL_Motor(delay_us);

    ALL_Motor_PD_Control(delay_us, init_angles);

    uint8_t channel;
    FrameInfo info_rx;
    uint8_t data_rx[8] = {0};

    for (size_t i = 0; i < 12; i++)
    {
    // 有数据不会阻塞，若无数据则等待1s
        int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e5);
    }
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

    Motor_PD_Control(USB2CAN0_, 2, 0x01, &Motor1_PDControl, -motor_angles[0]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 1, 0x04, &Motor2_PDControl, -motor_angles[3]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 2, 0x02, &Motor3_PDControl, -motor_angles[1]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 1, 0x05, &Motor4_PDControl, -motor_angles[4]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 2, 0x03, &Motor5_PDControl, -motor_angles[2]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor_PD_Control(USB2CAN0_, 1, 0x06, &Motor6_PDControl, -motor_angles[5]); // 电机顺时针为角度增加，所以加负号
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
    // Eigen::Quaternionf q_corrected = q_current * Q_offset;
    q_corrected.normalize();
    
    // 存回数据
    DEV0_RX.Module_CAN_Recieve[0].quat_w = q_corrected.w();
    DEV0_RX.Module_CAN_Recieve[0].quat_x = q_corrected.x();
    DEV0_RX.Module_CAN_Recieve[0].quat_y = q_corrected.y();
    DEV0_RX.Module_CAN_Recieve[0].quat_z = q_corrected.z();
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

