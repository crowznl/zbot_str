#include "Zbot_RL_4L.h"
#include <termios.h>

static class Hipnuc_IMU IMU;
static class RS_Motor Motor;

// 主函数循环
void Zbot_RL::Spin()
{
    while (!g_stop.load(std::memory_order_relaxed))
    {
        sleep(1); // 延时1s
    }
    printf("~ ALL Exit ~\n");
}

/// @brief 构造函数，初始化
/// @return
Zbot_RL::Zbot_RL()
{
    running_ = true;

    std::cout << std::endl
            << "RUN Zbot_RL.cpp" << std::endl
            << std::endl;

    USB2CAN0_ = openUSBCAN("/dev/USB2CAN0");
    if (USB2CAN0_ == -1){
        std::cout << std::endl
                << "USB2CAN0 open INcorrect!!!" << std::endl;
        exit(1);}
    else
        std::cout << std::endl
                << "USB2CAN0 opened ,num=" << USB2CAN0_ << std::endl;
    
    USB2CAN1_ = openUSBCAN("/dev/USB2CAN1");
    if (USB2CAN1_ == -1){
        std::cout << std::endl
                  << "USB2CAN1 open INcorrect!!!" << std::endl;
        closeUSBCAN(USB2CAN0_);
        exit(1);}
    else
        std::cout << std::endl
                  << "USB2CAN1 opened ,num=" << USB2CAN1_ << std::endl;

    USB2CAN2_ = openUSBCAN("/dev/USB2CAN2");
    if (USB2CAN2_ == -1){
        std::cout << std::endl
                  << "USB2CAN2 open INcorrect!!!" << std::endl;
        closeUSBCAN(USB2CAN0_);
        closeUSBCAN(USB2CAN1_);
        exit(1);}
    else
        std::cout << std::endl
                  << "USB2CAN2 opened ,num=" << USB2CAN2_ << std::endl;

    // 启动成功
    std::cout << std::endl
            << "USB2CAN   NODE INIT__OK   by TANGAIR" << std::endl
            << std::endl
            << std::endl;

    // ********************************************************************** 初 始 化 ********************************************************************** //
    {
        if constexpr (!motor_zero_set_already) // 电机未进行过零点设置
        {
            ALL_Motor_Zero_Set(Delay_1000us);
            sleep(1);
            Read_Clear(USB2CAN0_, 12); // 清理接收缓存
            Read_Clear(USB2CAN1_, 12);
            ALL_Motor_Angle_Read(Delay_1000us); // 读取当前电机角度检查

            std::cout << ">>> Zero Set Finished. Exiting Program... <<<" << std::endl;
            closeUSBCAN(USB2CAN0_); // 关闭CAN设备
            closeUSBCAN(USB2CAN1_);
            exit(0); // 直接终止进程，不再执行后续的线程创建，对象还没完全建立，析构函数也不会自动运行
            // exit(0) 或 exit(EXIT_SUCCESS) 表示程序正常终止
        }
        // 电机进行过零点设置
        if constexpr (Motor_Ctrl_Mode == PP_MODE)
        {
            ALL_Motor_PP_Init(init_angles); // 使能电机，切换到PP模式，运动到指定角度
            Read_Clear(USB2CAN0_, 30); // 清理接收缓存
            Read_Clear(USB2CAN1_, 30);
        }
        else if constexpr (Motor_Ctrl_Mode == PD_MODE)
        {
            ALL_Motor_PD_Init(init_angles);
            Read_Clear(USB2CAN0_, 12); // 清理接收缓存
            Read_Clear(USB2CAN1_, 12);
        }

        IMU.IMU_Set_SYNC_Mode(USB2CAN2_, 1, imu_module_id); // 设置IMU为SYNC模式
        // sleep(0.5);
        sleep(2); // 等待运动到初始位置

        for (int i = 0; i < 5; i++) // 采集多次初始IMU四元数数据
        {
            IMU.IMU_Send_SYNC(USB2CAN2_, 1);
            IMU.IMU_Get_init_quat(USB2CAN2_);
        }
        IMU.IMU_Get_offset_quat(Q_desired); // 计算校正四元数，Q_desired为期望初始四元数，此时朝向角为0
        sleep(1);
    }
    // ********************************************************************** 创 立 线 程 ********************************************************************** //
    if constexpr (sim_obs_play)
    {
        std::cout << ">>> SIM OBS PLAY MODE <<<" << std::endl;
        // 读取csv作为策略输入
        bool csv_ok = true;
        csv_data = loadCSV_invec(sim_obs_csv, csv_ok, observation_space);  // 好像搞那个布尔引用有点多余了，直接判断return 空 就行了。
        if (!csv_ok || csv_data.empty())
        {
            // std::cerr << "CSV 读取失败或数据异常，程序退出。" << std::endl;
            std::cout << "CSV 读取失败或数据异常，程序退出。" << std::endl;
            closeUSBCAN(USB2CAN0_);
            closeUSBCAN(USB2CAN1_);
            closeUSBCAN(USB2CAN2_);
            exit(1);
        }
        csv_index = 0;
    }

#ifdef USE_LIBTORCH
    // 加载模型，路径可在运行时替换
    try {
        policy_model = std::make_shared<torch::jit::script::Module>(torch::jit::load(model_path));
        model_loaded = true;
        std::cout << "Policy model loaded." << std::endl;
    } catch (const std::exception &e) {
        model_loaded = false;
        std::cout << "Warning: failed to load policy model: " << e.what() << std::endl;
        closeUSBCAN(USB2CAN0_);
        closeUSBCAN(USB2CAN1_);
        closeUSBCAN(USB2CAN2_);
        exit(1);
    }
#endif

    // 获取当前时间点，使用 steady_clock 避免系统时间跳变影响
    start_tp_ = std::chrono::steady_clock::now();

    // 创建CAN接收线程，设备0, 1, 2
    _CAN_RX_device_0_thread = std::thread(&Zbot_RL::CAN_RX_device_0_thread, this);
    _CAN_RX_device_1_thread = std::thread(&Zbot_RL::CAN_RX_device_1_thread, this);
    _CAN_RX_device_2_thread = std::thread(&Zbot_RL::CAN_RX_device_2_thread, this);

    // CAN发送线程
    _CAN_TX_thread = std::thread(&Zbot_RL::CAN_TX_thread, this);

    // 键盘输入线程
    _keyborad_input = std::thread(&Zbot_RL::keyborad_input, this);
    
    // 策略线程（50Hz）
    _strategy_thread = std::thread(&Zbot_RL::Strategy_thread, this);

    // 记录线程（50Hz）  //（500Hz）
    if constexpr (log_enabled) {
        _log_thread = std::thread(&Zbot_RL::Log_thread, this);
    }
}

/// @brief 析构函数
Zbot_RL::~Zbot_RL()
{

    running_ = false;

    /*注销线程*/

    // 当程序要退出时（析构里把 running_ = false;），主线程会 join() 等待键盘线程结束；
    // 但键盘线程此刻还卡在读取输入，不会回到 while (running_) 的判断，需要手动输入并按回车。只按回车一般不行，cin >> 会跳过空白（空格/换行）
    // 现在改用read()了，按任意键即可结束键盘线程。
    std::cout << std::endl
              << "----------------请任意输入，以结束键盘进程  ----------------------" << std::endl
              << std::endl;
   
    // can接收设备
    _CAN_RX_device_0_thread.join();
    _CAN_RX_device_1_thread.join();
    _CAN_RX_device_2_thread.join();

    //can发送测试线程
    _CAN_TX_thread.join();
    //键盘输入线程
    _keyborad_input.join();
    // 策略线程停止并join
    _strategy_thread.join();

    if (_log_thread.joinable()) {
        _log_thread.join();
    }
    // 失能电机
    ALL_Motor_DISABLE(100);

    // 关闭设备
    closeUSBCAN(USB2CAN0_);
    closeUSBCAN(USB2CAN1_);
    closeUSBCAN(USB2CAN2_);
}

/// @brief can设备0，接收线程函数
void Zbot_RL::CAN_RX_device_0_thread()
{
    can_dev0_rx_count = 0;
    can_dev0_rx_count_thread=0;
    
    while (running_)
    {

        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};
        // 统一暂存ID
        uint8_t TEMP_ID = 0;

        can_dev0_rx_count_thread++;

        // 有数据不会阻塞，若无数据则等待1s
        int recieve_re = readUSBCAN(USB2CAN0_, &channel, &info_rx, data_rx, 1e6);

        // 接收到数据
        if (recieve_re != -1)
        {
            can_dev0_rx_count++;

            if (info_rx.frameType == EXTENDED) // 读取电机数据
            {
                TEMP_ID = (info_rx.canID >> 8) & 0xff;
                // uint8_t index = TEMP_ID - 0x01;  // 将ID映射到0-5的索引,DEV0_RX包含ID01-ID06的电机数据
                uint8_t index = TEMP_ID - 0x0a;  // 将ID映射到0-5的索引,DEV0_RX包含ID10-ID15的电机数据

                {
                    std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
                    // 解码
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.master_id = (info_rx.canID) & 0xff;
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.motor_id = (info_rx.canID >> 8) & 0xff;
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.fault_message = (info_rx.canID >> 16) & 0x3f;
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.motor_state = (info_rx.canID >> 22) & 0x03;
                    DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.mode = (info_rx.canID >> 24) & 0x1f;

                    if (DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.mode == 0x02)
                    {
                        // 收到的数据高字节在前
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_position = (data_rx[0] << 8) | (data_rx[1]);
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_speed = (data_rx[2] << 8) | (data_rx[3]);
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_torque = (data_rx[4] << 8) | (data_rx[5]);
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_temp = (data_rx[6] << 8) | (data_rx[7]);

                        // 转换
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_position_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_position, (P_MIN), (P_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_speed_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_speed, (V_MIN), (V_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_torque_f = -uint_to_float(DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_torque, (T_MIN), (T_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_temp_f = (float)DEV0_RX.Module_CAN_Recieve[index].Motor_Recieve.current_temp / 10;
                    }
                }
            }

            // if constexpr (log_enabled && log_data_type == LOG_RX)
            // {   // it should be deprecated in future. 或者 ifdef DEBUG endif
            //     // 太不合理了，每来一个（电机）数据，就重新记录所有电机的数据。
            //     // 而且不好扩展，假如有多个USB2CAN设备呢，需要开启多个接收线程，难道分别搞几个 log_rx_n_queue 分开记录吗，也不好对齐时间。
            //     // log_rx_data.motor_pos[motor_dof] motor_dof>6 也不好扩展。每个接收线程只能更新自己负责的两条总线的模块数据。
            //     // 要不就定义不同的LogFrame，不再搞通用的了，那Log_thread()也得相应适配。
            //
            //     LogFrame log_rx_data;
            //     log_rx_data.timestamp = getTimestamp();
            //     {
            //         std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
            //         for (int i = 0; i < 6; ++i) {
            //             log_rx_data.motor_pos[i] = DEV0_RX.Module_CAN_Recieve[i].Motor_Recieve.current_position_f;
            //             log_rx_data.motor_vel[i] = DEV0_RX.Module_CAN_Recieve[i].Motor_Recieve.current_speed_f;
            //         }
            //     }
            //     {
            //         std::lock_guard<std::mutex> lock(mutex_log_rx_queue);
            //         log_rx_queue.push(log_rx_data);
            //     }
            // }
        }
    }
    std::cout << "CAN_RX_device_0_thread  Exit~~" << std::endl;
}

/// @brief can设备1，接收线程函数
void Zbot_RL::CAN_RX_device_1_thread()
{
    can_dev1_rx_count = 0;
    can_dev1_rx_count_thread=0;
    
    while (running_)
    {

        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};
        // 统一暂存ID
        uint8_t TEMP_ID = 0;

        can_dev1_rx_count_thread++;

        // 有数据不会阻塞，若无数据则等待1s
        int recieve_re = readUSBCAN(USB2CAN1_, &channel, &info_rx, data_rx, 1e6);

        // 接收到数据
        if (recieve_re != -1)
        {
            can_dev1_rx_count++;

            if (info_rx.frameType == EXTENDED) // 读取电机数据
            {
                TEMP_ID = (info_rx.canID >> 8) & 0xff;
                // uint8_t index = TEMP_ID - 0x07;  // 将ID映射到0-5的索引,DEV1_RX包含ID07-ID12的电机数据
                uint8_t index = TEMP_ID - 0x14;  // 将ID映射到0-5的索引,DEV1_RX包含ID20-ID25的电机数据

                {
                    std::lock_guard<std::mutex> lock(mutex_DEV1_RX);
                    // 解码
                    DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.master_id = (info_rx.canID) & 0xff;
                    DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.motor_id = (info_rx.canID >> 8) & 0xff;
                    DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.fault_message = (info_rx.canID >> 16) & 0x3f;
                    DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.motor_state = (info_rx.canID >> 22) & 0x03;
                    DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.mode = (info_rx.canID >> 24) & 0x1f;

                    if (DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.mode == 0x02)
                    {
                        // 收到的数据高字节在前
                        DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_position = (data_rx[0] << 8) | (data_rx[1]);
                        DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_speed = (data_rx[2] << 8) | (data_rx[3]);
                        DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_torque = (data_rx[4] << 8) | (data_rx[5]);
                        DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_temp = (data_rx[6] << 8) | (data_rx[7]);

                        // 转换
                        DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_position_f = -uint_to_float(DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_position, (P_MIN), (P_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_speed_f = -uint_to_float(DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_speed, (V_MIN), (V_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_torque_f = -uint_to_float(DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_torque, (T_MIN), (T_MAX), 16); // 电机顺时针为角度增加，所以加负号
                        DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_temp_f = (float)DEV1_RX.Module_CAN_Recieve[index].Motor_Recieve.current_temp / 10;
                    }
                }
            }
        }
    }
    std::cout << "CAN_RX_device_1_thread  Exit~~" << std::endl;
}

/// @brief can设备2，接收线程函数
void Zbot_RL::CAN_RX_device_2_thread()
{
    can_dev2_rx_count = 0;
    can_dev2_rx_count_thread=0;
    
    while (running_)
    {

        uint8_t channel;
        FrameInfo info_rx;
        uint8_t data_rx[8] = {0};
        // 统一暂存ID
        uint8_t TEMP_ID = 0;

        can_dev2_rx_count_thread++;

        // 有数据不会阻塞，若无数据则等待1s
        int recieve_re = readUSBCAN(USB2CAN2_, &channel, &info_rx, data_rx, 1e6);

        // 接收到数据
        if (recieve_re != -1)
        {
            can_dev2_rx_count++;

            if (info_rx.frameType == STANDARD) // 读取IMU数据
            {
                TEMP_ID = ((info_rx.canID) & 0xfff) - 0x480;
                uint8_t index = TEMP_ID - imu_module_id;  // 将ID映射到0开始的索引,DEV0_RX包含ID21的IMU数据

                {
                    std::lock_guard<std::mutex> lock(mutex_DEV2_RX);
                    // 收到的数据低字节在前
                    DEV2_RX.Module_CAN_Recieve[index].IMU_Recieve.quat[0] = static_cast<float>(static_cast<int16_t>((data_rx[1] << 8) | data_rx[0])) / 10000.0f;
                    DEV2_RX.Module_CAN_Recieve[index].IMU_Recieve.quat[1] = static_cast<float>(static_cast<int16_t>((data_rx[3] << 8) | data_rx[2])) / 10000.0f;
                    DEV2_RX.Module_CAN_Recieve[index].IMU_Recieve.quat[2] = static_cast<float>(static_cast<int16_t>((data_rx[5] << 8) | data_rx[4])) / 10000.0f;
                    DEV2_RX.Module_CAN_Recieve[index].IMU_Recieve.quat[3] = static_cast<float>(static_cast<int16_t>((data_rx[7] << 8) | data_rx[6])) / 10000.0f;
                }
            }
        }
    }
    std::cout << "CAN_RX_device_2_thread  Exit~~" << std::endl;
}

// can发送线程函数
void Zbot_RL::CAN_TX_thread()
{
    // 发送计数
    uint32_t tx_count = 0;

    while (running_)
    {
        // CAN发送计数
        tx_count++;

        IMU.IMU_Send_SYNC(USB2CAN2_, 1); // 发送IMU同步帧
        std::this_thread::sleep_for(std::chrono::microseconds(Delay_300us)); // 单位us

        // 拷贝策略线程计算得到的输出
        {
            std::lock_guard<std::mutex> lock(mutex_position_output); //线程锁
            for (size_t i = 0; i < motor_angles.size(); ++i) {
                motor_angles[i] = position_output[i];
            }
        }

        for (size_t i = 0; i < motor_angles.size(); ++i) {
            motor_angles[i] = std::clamp(motor_angles[i], lower_limit[i], upper_limit[i]);
        }

        if constexpr (Motor_Ctrl_Mode == PD_MODE)
        {
            ALL_Motor_PD_Control(Delay_300us, motor_angles); // <<================================================================ 测试一下Delay多少合适
            // ALL_Motor_PD_Control(Delay_300us, init_angles);
        }
        else if constexpr (Motor_Ctrl_Mode == PP_MODE)
        {
            ALL_Motor_PP_Angle_Set(Delay_300us, motor_angles);
            // ALL_Motor_PP_Angle_Set(Delay_300us, init_angles);
        }

        // if constexpr (log_enabled && log_data_type == LOG_TX)
        // {   // it should be deprecated in future. 或者 ifdef DEBUG endif
        //     // 也不合理，只发送位置指令，却记录多余数据的默认值。而且位置指令只会在策略线程（50Hz）更改时变化，没必要每次发送都记录。
        //     LogFrame log_tx_data;
        //     log_tx_data.timestamp = getTimestamp();
        //     // 记录下发的所有电机目标位置
        //     for (int i = 0; i < motor_dof; ++i)
        //     {
        //         log_tx_data.motor_pos[i] = motor_angles[i];
        //         log_tx_data.motor_vel[i] = 0.0f; // TX通常是位置指令，速度未知或设为0
        //     }
        //     // TX线程不直接读取IMU，设为默认
        //     log_tx_data.imu_quat[0] = 1.0f; log_tx_data.imu_quat[1] = 0.0f;
        //     log_tx_data.imu_quat[2] = 0.0f; log_tx_data.imu_quat[3] = 0.0f;

        //     {
        //         std::lock_guard<std::mutex> lock(mutex_log_tx_queue);
        //         log_tx_queue.push(log_tx_data);
        //     }
        // }

        // 打印数据
        if (tx_count % 10000 == 0)
        {
            // 理论上position_output和motor_angles是一致的，只不过motor_angles加的限幅可能和position_output（relative_vec）不一致。
            // for (size_t i = 0; i < motor_angles.size(); ++i) {
            //     std::cout << " 输入电机" << (i+1) << "的角度: " << motor_angles[i] << std::endl;
            // }
            std::cout << " 发送次数:               " << tx_count << std::endl
                      << " CAN转USB设备0 接收次数: " << can_dev0_rx_count << std::endl
                      << " CAN转USB设备1 接收次数: " << can_dev1_rx_count << std::endl
                      << " CAN转USB设备2 接收次数: " << can_dev2_rx_count << std::endl
                      << " TIME:                  " << getTimestamp() << "s" << std::endl
                      << " ***************************************************************** " << std::endl;
        }
    }

    //程序终止时的提示信息
    std::cout << "CAN_TX_test_thread  Exit~~" << std::endl;
}

// 键盘输入线程
void Zbot_RL::keyborad_input()
{
    struct termios oldt, newt;
    // 获取当前终端属性
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    // 关闭规范模式(ICANON)和回显(ECHO)
    newt.c_lflag &= ~(ICANON | ECHO);
    // 未修改 VMIN/VTIME，默认情况下，不按键时不耗 CPU。终端设置通常是 VMIN=1, VTIME=0 (完全阻塞模式)：
    // read 会一直阻塞（睡觉），直到读取到至少 VMIN 个字符才返回。
    // 设置新的终端属性
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    std::cout << "Keyboard control enabled (W/S: Velocity, A/D: Yaw)." << std::endl;
    // std::cout << std::fixed << std::setprecision(2);

    while (running_)
    {
        char c;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n > 0)
        {
            if (!running_) break;

            std::lock_guard<std::mutex> lock(mutex_keyboard_input);
            bool updated = false;

            if (c == 'W' || c == 'w') {
                command_1 += 0.05f;
                updated = true;
            }
            else if (c == 'S' || c == 's') {
                command_1 -= 0.05f;
                updated = true;
            }
            else if (c == 'A' || c == 'a') {
                target_heading_yaw += 0.05f;
                updated = true;
            }
            else if (c == 'D' || c == 'd') {
                target_heading_yaw -= 0.05f;
                updated = true;
            }
            else if (c == 'P' || c == 'p') {
                print_info_flag = !print_info_flag;
                std::cout << "\nPrint Info: " << (print_info_flag ? "ON" : "OFF") << std::endl;
            }

            if (updated) {
                std::cout << "Velocity X: " << command_1 
                          << ", Target Yaw: " << target_heading_yaw << std::endl;
            } // 如需只在同一行更新数值，不反复刷屏，去掉 << std::endl，改用回车符<< "      \r" << std::flush;
        }
        else if (n <= 0)
        {
             // EOF or Error
             break;
        }
    }
    
    // 恢复终端属性
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    std::cout << "keyboard_input_thread  Exit~~" << std::endl;
}

// 策略线程：以50Hz运行，读取 keyboard_input 与 zbot 状态，计算 position_output
void Zbot_RL::Strategy_thread()
{
    const std::chrono::milliseconds period(20); // 50Hz
    // const std::chrono::milliseconds period(100); // 10Hz

    std::vector<float> cur_pos(motor_dof, 0.0f);
    std::vector<float> cur_vel(motor_dof, 0.0f);
    float cur_quat_w = Q_desired.w();
    float cur_quat_x = Q_desired.x();
    float cur_quat_y = Q_desired.y();
    float cur_quat_z = Q_desired.z();
    float command1;
    float heading_err;  // TODO: 计算航向误差
    bool print_info = false;

    while (running_)
    {
        auto t0 = std::chrono::steady_clock::now();

        // 读取必要数据并计算输出
        {
            std::lock_guard<std::mutex> lock(mutex_DEV0_RX);
            for (int i = 0; i < 6; ++i) {
                 cur_pos[i] = DEV0_RX.Module_CAN_Recieve[i].Motor_Recieve.current_position_f;
                 cur_vel[i] = DEV0_RX.Module_CAN_Recieve[i].Motor_Recieve.current_speed_f;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_DEV1_RX);
            for (int i = 0; i < 6; ++i) {
                 cur_pos[i+6] = DEV1_RX.Module_CAN_Recieve[i].Motor_Recieve.current_position_f;
                 cur_vel[i+6] = DEV1_RX.Module_CAN_Recieve[i].Motor_Recieve.current_speed_f;
            }
        }
        {
            std::lock_guard<std::mutex> lock(mutex_DEV2_RX);
            IMU.IMU_quat_correct(DEV2_RX.Module_CAN_Recieve[0].IMU_Recieve); // 四元数校正

            cur_quat_w = DEV2_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[0];
            cur_quat_x = DEV2_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[1];
            cur_quat_y = DEV2_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[2];
            cur_quat_z = DEV2_RX.Module_CAN_Recieve[0].IMU_Recieve.quat[3];
        }
        // std::cout << " IMU四元数: [" << cur_quat_w << "," << cur_quat_x << "," << cur_quat_y << "," << cur_quat_z << "]" << std::endl;

#ifdef USE_LIBTORCH
        if (model_loaded && policy_model)
        {
            try {
                std::vector<float> invec;
                
                if constexpr (!sim_obs_play)
                {   // 构造输入tensor：模型接受输入

                    // if constexpr (observation_space == 24){
                    //     invec = {cur_quat_w, cur_quat_x, cur_quat_y, cur_quat_z, 
                    //              cur_pos[0] - init_angles[0], cur_pos[1] - init_angles[1], cur_pos[2] - init_angles[2], cur_pos[3] - init_angles[3], cur_pos[4] - init_angles[4], cur_pos[5] - init_angles[5],
                    //              cur_vel[0], cur_vel[1], cur_vel[2], cur_vel[3], cur_vel[4], cur_vel[5],
                    //              out_last[0], out_last[1], out_last[2], out_last[3], out_last[4], out_last[5],
                    //              command1,
                    //             //  heading_err
                    //             };
                    // }else if constexpr (observation_space == 23){}

                    // ---- 自动拼接策略输入 ----
                    invec.reserve(static_cast<size_t>(observation_space));

                    // IMU 四元数 (w,x,y,z)
                    invec.push_back(cur_quat_w);
                    invec.push_back(cur_quat_x);
                    invec.push_back(cur_quat_y);
                    invec.push_back(cur_quat_z);

                    // 电机相对位置/速度（按 motor_dof 数量）
                    // for (size_t i = 0; i < static_cast<size_t>(motor_dof); ++i)
                    // {
                    //     invec.push_back(cur_pos[i] - init_angles[i]);
                    // }
                    // for (size_t i = 0; i < static_cast<size_t>(motor_dof); ++i)
                    // {
                    //     invec.push_back(cur_vel[i]);
                    // }
                    for (size_t i = 0; i < 4; ++i)
                    {
                        invec.push_back(cur_pos[i] - init_angles[i]);
                        invec.push_back(cur_pos[i+3] - init_angles[i+3]);
                        invec.push_back(cur_pos[i+6] - init_angles[i+6]);
                        invec.push_back(cur_pos[i+9] - init_angles[i+9]);
                    }
                    for (size_t i = 0; i < 4; ++i)
                    {
                        invec.push_back(cur_vel[i]);
                        invec.push_back(cur_vel[i+3]);
                        invec.push_back(cur_vel[i+6]);
                        invec.push_back(cur_vel[i+9]);
                    }

                    // 上一次动作 out_last（按 action_space）
                    for (size_t i = 0; i < static_cast<size_t>(action_space); ++i)
                    {
                        invec.push_back(out_last[i]);
                    }

                    if constexpr (obs_include_heading_err)
                    {   // 计算heading_err
                        // Eigen::Quaternionf q_current(cur_quat_w, cur_quat_x, cur_quat_y, cur_quat_z);
                        // Eigen::Vector3f axis_x(1.0f, 0.0f, 0.0f);
                        // Eigen::Vector3f base_dir_forward_w = q_current * axis_x;
                        // float current_yaw = std::atan2(base_dir_forward_w.y(), base_dir_forward_w.x());
                        // 直接使用四元数转Yaw角公式: atan2(2(wz + xy), 1 - 2(y^2 + z^2))
                        float current_yaw = std::atan2(2.0f * (cur_quat_w * cur_quat_z + cur_quat_x * cur_quat_y),
                                                       1.0f - 2.0f * (cur_quat_y * cur_quat_y + cur_quat_z * cur_quat_z));

                        float diff;
                        {
                            std::lock_guard<std::mutex> lock(mutex_keyboard_input); 
                            command1 = command_1;
                            diff = target_heading_yaw - current_yaw;
                            print_info = print_info_flag;
                        }
                        heading_err = std::atan2(std::sin(diff), std::cos(diff));

                        invec.push_back(command1);
                        invec.push_back(heading_err);
                    }
                    else if constexpr (obs_include_command)
                    {
                        {
                            std::lock_guard<std::mutex> lock(mutex_keyboard_input); 
                            command1 = command_1;
                            print_info = print_info_flag;
                        }
                        invec.push_back(command1);
                    }
                    else
                    {
                        {
                            std::lock_guard<std::mutex> lock(mutex_keyboard_input); 
                            print_info = print_info_flag;
                        }
                    }
                }
                else
                {   
                    {
                        std::lock_guard<std::mutex> lock(mutex_keyboard_input); 
                        print_info = print_info_flag;
                    }
                    // ---- 读取CSV作为策略输入 ----
                    if (csv_index < csv_data.size()) {
                        invec = csv_data[csv_index++];
                    } else {
                        // 播放完就停在最后一行 // 停不住：虽然out不变,relative_tensor一直增加
                        // invec = csv_data.back();
                        std::cout << "CSV EOF" << std::endl;
                        return;
                    }
                }

                at::Tensor input_tensor = torch::from_blob(invec.data(), {1, static_cast<int64_t>(invec.size())}).clone();
                // 前向推理
                std::vector<torch::jit::IValue> inputs;
                inputs.push_back(input_tensor);
                at::Tensor out = policy_model->forward(inputs).toTensor().squeeze().tanh();

                out_last.assign(out.data_ptr<float>(), 
                                out.data_ptr<float>() + out.numel());  // 保存本次输出，作为下次输入的一部分

                relative_tensor += out * joint_speed_limit * PI * 0.02f;
                relative_tensor = relative_tensor.clip(-1.0f * PI, 1.0f * PI);
                // at::Tensor类型没有toVector
                std::vector<float> relative_vec(relative_tensor.data_ptr<float>(), 
                                              relative_tensor.data_ptr<float>() + relative_tensor.numel());
                {
                    std::lock_guard<std::mutex> lock(mutex_position_output);
                    for (size_t i = 0; i < relative_vec.size(); ++i) {
                        position_output[i] = relative_vec[i] + init_angles[i];
                    }
                }

                if (print_info)
                {
                    for (size_t i = 0; i < static_cast<size_t>(motor_dof); ++i) {
                        std::cout << " 当前电机" << (i+1) << "的角度: " << cur_pos[i] << std::endl;
                        std::cout << " 策略输出" << (i+1) << "的角度: " << relative_vec[i] + init_angles[i] << std::endl;
                        // 理论上position_output和motor_angles是一致的，
                        // 只不过motor_angles加的限幅可能和position_output（relative_tensor）的不一致。
                    }
                    {
                        std::lock_guard<std::mutex> lock(mutex_keyboard_input); 
                        print_info_flag = !print_info_flag;
                    }
                }

                if constexpr (log_enabled && log_data_type == LOG_STRATEGY)
                {
                    LogFrame log_strategy_data;
                    log_strategy_data.timestamp = getTimestamp();
                    for (size_t i = 0; i < static_cast<size_t>(motor_dof); ++i) {
                        // log_strategy_data.motor_pos[i] = cur_pos[i] - init_angles[i];
                        log_strategy_data.motor_pos[i] = relative_vec[i];
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
                {
                    std::lock_guard<std::mutex> lock(mutex_position_output);
                    position_output = init_angles;
                }
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
void Zbot_RL::Log_thread() {
    std::string log_path = log_file_path;
    
    // 确保路径以斜杠结尾
    if (!log_path.empty() && log_path.back() != '/') {
        log_path += '/';
    }
    
    using namespace std::chrono;
    auto start_time = steady_clock::now();
    int file_index = 0;
    std::ofstream log_file;
    std::string prefix;

    if constexpr (log_data_type == LOG_STRATEGY) prefix = "log_strategy_";
    else if constexpr (log_data_type == LOG_TX) prefix = "log_tx_";
    else if constexpr (log_data_type == LOG_RX) prefix = "log_rx_";

    auto openNewFile = [&](int index) {
        if (log_file.is_open()) log_file.close();

        std::string filename = log_path + prefix + std::to_string(index) + ".csv";
        log_file.open(filename, std::ios::out);
        if (!log_file.is_open()) {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return;
        }
        // 表头：四元数在前
        log_file << "timestamp,imu_w,imu_x,imu_y,imu_z";
        for (int i = 0; i < motor_dof; ++i) log_file << ",pos" << (i + 1);
        for (int i = 0; i < motor_dof; ++i) log_file << ",vel" << (i + 1);
        log_file << "\n";
        
        std::cout << "创建日志文件: " << filename << std::endl;
    };

    openNewFile(file_index);

    while (running_) {
        auto now = steady_clock::now();
        if (duration_cast<seconds>(now - start_time).count() >= 100) {
            file_index++;
            openNewFile(file_index);
            start_time = now;
        }

        // 批量收集队列数据
        std::vector<LogFrame> batch;
        
        if constexpr (log_data_type == LOG_STRATEGY) {
             std::lock_guard<std::mutex> lock(mutex_log_strategy_queue);
             while (!log_strategy_queue.empty()) {
                 batch.push_back(log_strategy_queue.front());
                 log_strategy_queue.pop();
             }
        } 
        // else if constexpr (log_data_type == LOG_TX) {
        //      std::lock_guard<std::mutex> lock(mutex_log_tx_queue);
        //      while (!log_tx_queue.empty()) {
        //          batch.push_back(log_tx_queue.front());
        //          log_tx_queue.pop();
        //      }
        // } 
        // else if constexpr (log_data_type == LOG_RX) {
        //      std::lock_guard<std::mutex> lock(mutex_log_rx_queue);
        //      while (!log_rx_queue.empty()) {
        //          batch.push_back(log_rx_queue.front());
        //          log_rx_queue.pop();
        //      }
        // }

        // 批量写入日志
        if (!batch.empty() && log_file.is_open()) {
            for (auto &entry : batch) {
                log_file << std::fixed << std::setprecision(6) << entry.timestamp << ",";
                
                // 先写入 IMU 四元数 (w,x,y,z)
                for (int i = 0; i < 4; ++i) {
                    log_file << entry.imu_quat[i] << ",";
                }
                
                // 再写入电机位置
                for (int i = 0; i < motor_dof; ++i) {
                    log_file << entry.motor_pos[i] << ",";
                }
                
                // 最后写入电机速度（最后一个不加逗号）
                for (int i = 0; i < motor_dof; ++i) {
                    log_file << entry.motor_vel[i];
                    if (i < motor_dof - 1) {
                        log_file << ",";
                    } else {
                        log_file << "\n";
                    }
                }
            }
            log_file.flush();  // 确保数据写入磁盘
        }

        // 队列为空则短暂休眠
        if (batch.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    if (log_file.is_open()) log_file.close();
    
    std::cout << "日志线程结束，文件保存路径: " << log_path << std::endl;
}

void Zbot_RL::ALL_Motor_ENABLE(int delay_us)
{
    Motor.Motor_Enable(USB2CAN0_, 2, 0x01);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN1_, 2, 0x07);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 1, 0x04);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN1_, 1, 0x10);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 2, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN1_, 2, 0x08);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 1, 0x05);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN1_, 1, 0x11);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 2, 0x03);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN1_, 2, 0x09);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN0_, 1, 0x06);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Enable(USB2CAN1_, 1, 0x12);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Zbot_RL::ALL_Motor_DISABLE(int delay_us)
{   
    Motor.Motor_Disable(USB2CAN0_, 2, 0x01);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN1_, 2, 0x07);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 1, 0x04);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN1_, 1, 0x10);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 2, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN1_, 2, 0x08);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 1, 0x05);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN1_, 1, 0x11);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 2, 0x03);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN1_, 2, 0x09);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN0_, 1, 0x06);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Disable(USB2CAN1_, 1, 0x12);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Zbot_RL::ALL_Motor_PP_Mode_Set(int delay_us)
{
    Motor.PP_Mode_Set(USB2CAN0_, 2, 0x01, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN1_, 2, 0x07, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 1, 0x04, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN1_, 1, 0x10, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 2, 0x02, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN1_, 2, 0x08, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 1, 0x05, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN1_, 1, 0x11, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 2, 0x03, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN1_, 2, 0x09, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN0_, 1, 0x06, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Mode_Set(USB2CAN1_, 1, 0x12, 20.0f, 30.0f, delay_us);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}

void Zbot_RL::ALL_Motor_PP_Angle_Set(int delay_us, std::vector<float> motor_angles)
{
    Motor.PP_Angle_Set(USB2CAN0_, 2, 0x01, -motor_angles[0]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN1_, 2, 0x07, -motor_angles[6]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 1, 0x04, -motor_angles[3]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN1_, 1, 0x10, -motor_angles[9]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 2, 0x02, -motor_angles[1]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN1_, 2, 0x08, -motor_angles[7]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 1, 0x05, -motor_angles[4]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN1_, 1, 0x11, -motor_angles[10]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 2, 0x03, -motor_angles[2]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN1_, 2, 0x09, -motor_angles[8]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN0_, 1, 0x06, -motor_angles[5]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));

    Motor.PP_Angle_Set(USB2CAN1_, 1, 0x12, -motor_angles[11]); // 电机顺时针为角度增加，所以加负号
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}

void Zbot_RL::ALL_Motor_PP_Init(std::vector<float> motor_angles)
{
    // 所有电机设置为PP模式
    ALL_Motor_PP_Mode_Set(Delay_10000us);
    sleep(0.1);

    // 以PP模式运动到初始位置
    ALL_Motor_PP_Angle_Set(Delay_10000us, motor_angles);
    sleep(3);
}

void Zbot_RL::ALL_Motor_Zero_Set(int delay_us)
{
    Motor.Motor_Zero_Set(USB2CAN0_, 2, 0x01);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN1_, 2, 0x07);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 1, 0x04);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN1_, 1, 0x10);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 2, 0x02);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN1_, 2, 0x08);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 1, 0x05);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN1_, 1, 0x11);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 2, 0x03);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN1_, 2, 0x09);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN0_, 1, 0x06);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    Motor.Motor_Zero_Set(USB2CAN1_, 1, 0x12);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Zbot_RL::ALL_Motor_Angle_Read(int delay_us)
{
    std::cout << "Motor 01 Angle: " << Motor.Angle_Read(USB2CAN0_, 2, 0x01) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 07 Angle: " << Motor.Angle_Read(USB2CAN1_, 2, 0x07) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 04 Angle: " << Motor.Angle_Read(USB2CAN0_, 1, 0x04) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 10 Angle: " << Motor.Angle_Read(USB2CAN1_, 1, 0x10) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 02 Angle: " << Motor.Angle_Read(USB2CAN0_, 2, 0x02) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 08 Angle: " << Motor.Angle_Read(USB2CAN1_, 2, 0x08) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 05 Angle: " << Motor.Angle_Read(USB2CAN0_, 1, 0x05) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 11 Angle: " << Motor.Angle_Read(USB2CAN1_, 1, 0x11) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 03 Angle: " << Motor.Angle_Read(USB2CAN0_, 2, 0x03) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 09 Angle: " << Motor.Angle_Read(USB2CAN1_, 2, 0x09) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us 

    std::cout << "Motor 06 Angle: " << Motor.Angle_Read(USB2CAN0_, 1, 0x06) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us

    std::cout << "Motor 12 Angle: " << Motor.Angle_Read(USB2CAN1_, 1, 0x12) << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); // 单位us
}

void Zbot_RL::ALL_Motor_PD_Init(std::vector<float> motor_angles)
{
    ALL_Motor_ENABLE(Delay_1000us); // 使能电机
    sleep(1);
    ALL_Motor_PD_Control(Delay_1000us, motor_angles); // PD模式运动到指定角度
    sleep(3);
}

void Zbot_RL::ALL_Motor_PD_Control(int delay_us, std::vector<float> motor_angles)
{
    auto t = std::chrono::high_resolution_clock::now();//这一句耗时50us

    // Motor.Motor_PD_Control(USB2CAN0_, 2, 0x01, &Zbot_RL_4L_PD, -motor_angles[0]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN1_, 2, 0x07, &Zbot_RL_4L_PD, -motor_angles[6]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN0_, 1, 0x04, &Zbot_RL_4L_PD, -motor_angles[3]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN1_, 1, 0x10, &Zbot_RL_4L_PD, -motor_angles[9]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN0_, 2, 0x02, &Zbot_RL_4L_PD, -motor_angles[1]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN1_, 2, 0x08, &Zbot_RL_4L_PD, -motor_angles[7]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN0_, 1, 0x05, &Zbot_RL_4L_PD, -motor_angles[4]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN1_, 1, 0x11, &Zbot_RL_4L_PD, -motor_angles[10]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN0_, 2, 0x03, &Zbot_RL_4L_PD, -motor_angles[2]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN1_, 2, 0x09, &Zbot_RL_4L_PD, -motor_angles[8]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN0_, 1, 0x06, &Zbot_RL_4L_PD, -motor_angles[5]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    // Motor.Motor_PD_Control(USB2CAN1_, 1, 0x12, &Zbot_RL_4L_PD, -motor_angles[11]); // 电机顺时针为角度增加，所以加负号
    // t += std::chrono::microseconds(delay_us);
    // std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 1, 0x0a, &Zbot_RL_4L_PD, -motor_angles[0]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN1_, 1, 0x14, &Zbot_RL_4L_PD, -motor_angles[2]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 2, 0x0d, &Zbot_RL_4L_PD, -motor_angles[1]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN1_, 2, 0x17, &Zbot_RL_4L_PD, -motor_angles[3]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 1, 0x0b, &Zbot_RL_4L_PD, -motor_angles[4]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN1_, 1, 0x15, &Zbot_RL_4L_PD, -motor_angles[6]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 2, 0x0e, &Zbot_RL_4L_PD, -motor_angles[5]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN1_, 2, 0x18, &Zbot_RL_4L_PD, -motor_angles[7]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
    
    Motor.Motor_PD_Control(USB2CAN0_, 1, 0x0c, &Zbot_RL_4L_PD, -motor_angles[8]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN1_, 1, 0x16, &Zbot_RL_4L_PD, -motor_angles[10]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN0_, 2, 0x0f, &Zbot_RL_4L_PD, -motor_angles[9]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);

    Motor.Motor_PD_Control(USB2CAN1_, 2, 0x19, &Zbot_RL_4L_PD, -motor_angles[11]); // 电机顺时针为角度增加，所以加负号
    t += std::chrono::microseconds(delay_us);
    std::this_thread::sleep_until(t);
}

void Zbot_RL::Read_Clear(int dev, int num)
{
    uint8_t channel;
    FrameInfo info_rx;
    uint8_t data_rx[8] = {0};

    for (int i = 0; i < num; i++)
    {
        // 有数据不会阻塞，若无数据则等待0.1s
        readUSBCAN(dev, &channel, &info_rx, data_rx, 1e5);
    }
}

float Zbot_RL::getTimestamp() {
    // 获取当前时间戳
    const auto elapsed = std::chrono::steady_clock::now() - start_tp_;
    return std::chrono::duration<float>(elapsed).count();
}

std::vector<std::vector<float>> loadCSV_invec(const std::string& path, bool& load_ok, int expected_cols) {
    load_ok = false;

    std::vector<std::vector<float>> data;
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "Error: 无法打开CSV文件: " << path << std::endl;
        // return std::vector<std::vector<float>>{};
        return {};  // 等价
    }

    std::string line;
    bool is_first_line = true;
    int line_num = 0;

    while (std::getline(file, line))
    {
        line_num++;

        // 跳过表头
        if (is_first_line)
        {
            is_first_line = false;
            continue;
        }

        std::stringstream ss(line);
        std::string cell;

        std::vector<float> row_vec;
        int col_index = 0;

        while (std::getline(ss, cell, ','))
        {
            // 去掉前后空白和 \r
            cell.erase(0, cell.find_first_not_of(" \t\r"));
            cell.erase(cell.find_last_not_of(" \t\r") + 1);

            // 第一列是时间戳，从第2列开始读取：col_index 1 ~ expected_cols
            if (col_index >= 1 && col_index <= expected_cols)
            {
                if (!cell.empty())
                {
                    try
                    {
                        row_vec.push_back(std::stof(cell));
                    }
                    catch (...)
                    {
                        std::cerr << "Error: CSV 第 " << line_num
                                  << " 行第 " << (col_index + 1)
                                  << " 列无法转换为数字，值= " << cell
                                  << " ，停止读取。\n";
                        return {};
                    }
                }
                else
                {
                    std::cerr << "Error: CSV 第 " << line_num
                              << " 行第 " << (col_index + 1)
                              << " 列为空" << "，停止读取。\n";
                    return {};
                }
            }

            col_index++;
        }

        // 必须是 expected_cols 列（不满足视为异常，直接停止）
        if (static_cast<int>(row_vec.size()) != expected_cols)
        {
            std::cerr << "Error: CSV 第 " << line_num
                      << " 行有效列数不是" << expected_cols << " （实际=" << row_vec.size()
                      << "），停止读取。\n";
            return {};
        }

        data.push_back(row_vec);
    }

    load_ok = true;
    return data;
}


