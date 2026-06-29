#include <iostream>
#include "FSM/State_VRL.h"
#include <fstream>
#include <algorithm>
#include <visualization_msgs/Marker.h>

State_VRL::State_VRL(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp,FSMStateName::VRL,"vrl"),
    _est(ctrlComp->estimator),_contact(ctrlComp->contact),
    _robModel(ctrlComp->robotModel),
    _has_map(false){
        //向物理模型询问这只机器狗的性能极限
        this->_vxLim = _robModel->getRobVelLimitX();  // 速度限幅
        this->_vyLim = _robModel->getRobVelLimitY();
        this->_wyawLim = _robModel->getRobVelLimitYaw();

        this->_scan_data.assign(187, 0.0f);
        ros::NodeHandle nh;
        // 创建 TF 监听器，订阅 /tf 和 /tf_static 话题，向 _tf_buffer 填充坐标变换数据
        _tf_listener = new tf2_ros::TransformListener(_tf_buffer);
        //订阅高程图的数据 elevation_map_raw  elevation_map
        _elevation_sub = nh.subscribe("/elevation_mapping/elevation_map_raw", 1,
                                  &State_VRL::mapCallback, this);
        std::cout << "[VRL State] Elevation Mapping Subscriber Initialized!" << std::endl;

        sampled_points_pub_ = nh.advertise<visualization_msgs::Marker>("sampled_height_points", 1);
    }

void State_VRL::mapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
    // 互斥锁保护：防止多线程同时读写地图导致内存崩溃 当执行完大括号内的内容自动关闭
    std::lock_guard<std::mutex> lock(_map_mutex);
    
    // 将 ROS 消息解码为 C++ 的 grid_map 对象
    grid_map::GridMapRosConverter::fromMessage(*msg, this->_current_map);
    this->_has_map = true;
}

void State_VRL::updateScanData()
{
    std::lock_guard<std::mutex> lock(_map_mutex);
    
    // 1. 如果还没收到第一帧地图，保持全 0 默认值（平地），直接返回
    if (!this->_has_map) {
        return; 
    }

    // 2. 清空上一帧的旧高度数据
    this->_scan_data.clear();
    float robot_x   = 0.0f; 
    float robot_y   = 0.0f;
    float robot_z   = 0.0f;
    float robot_yaw = 0.0f; 

    try {
        // 向 ROS TF 树索要从 "map" (世界原点) 到 "trunk" (机器狗中心) 的最新坐标变换
        //获取机械狗（trunk）在世界系(map)下的绝对位姿
        geometry_msgs::TransformStamped transformStamped = 
            _tf_buffer.lookupTransform("map", "trunk", ros::Time(0));
            
        // 提取 XYZ 绝对平移坐标
        robot_x = transformStamped.transform.translation.x;
        robot_y = transformStamped.transform.translation.y;
        robot_z = transformStamped.transform.translation.z;
        
        // 提取旋转四元数并转换为欧拉角 (Roll, Pitch, Yaw)
        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        robot_yaw = yaw; // 拿到弧度制的航向角
        
    } catch (tf2::TransformException &ex) {
        // 防护机制：如果系统刚启动还没建好图，或者 TF 树断了，打印警告并直接退出本次查图
        // 使用 ROS_WARN_THROTTLE 限制每 1 秒最多打印一次，防止刷屏导致系统卡顿
        ROS_WARN_THROTTLE(1.0, "[VRL State] TF 变换获取失败: %s", ex.what());
        return; 
    }

    const std::string layer = "elevation"; // 高程图对应的图层名 高度层

    // ================= 1. 初始化可视化 Marker =================
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = "map"; // ⚠️ 这里填你世界坐标系的 frame_id，通常是 "map" 或 "odom"
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "height_sampling";
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.id = 0;
    points_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    // 设置采样点的大小 (比如 3 厘米的红色小球)
    points_marker.scale.x = 0.03; 
    points_marker.scale.y = 0.03;
    points_marker.scale.z = 0.03;
    points_marker.color.r = 1.0f; // 红色
    points_marker.color.g = 0.0f;
    points_marker.color.b = 0.0f;
    points_marker.color.a = 1.0f; // ⚠️ 透明度必须大于0，否则看不见！
    //在PyTorch中torch.meshgrid(x,y)默认使用 ij 索引:x外循环 y内循环
    // 4. 遍历并采样 187 个点
    for (int row = 0; row < 17; row++) {
        for (int col = 0; col < 11; col++) {
            // 计算当前采样点相对于机器狗中心的局部坐标 (举例：以狗为中心前后左右辐射)
            // measured_points_x = [-0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7,0.8]
            // measured_points_y = [-0.5, -0.4, -0.3, -0.2, -0.1, 0., 0.1, 0.2, 0.3, 0.4, 0.5]
            float local_x = (row - 8) * 0.1f; 
            float local_y = (col - 5) * 0.1f;

            // 5. 坐标旋转与平移：局部坐标 (local) ➡️ 世界坐标 (global) 将狗的第一视角看到的点转到上帝视角地图上
            float global_x = robot_x + local_x * cos(robot_yaw) - local_y * sin(robot_yaw);
            float global_y = robot_y + local_x * sin(robot_yaw) + local_y * cos(robot_yaw);

            grid_map::Position position(global_x, global_y);
            float relative_height = 0.0f;

            // 6. 查表获取地图绝对高度
            // float map_absolute_z = 0.0f; // 🌟 黄金法则：如果看不见/出界，统统假设是海拔 0.0 的平地
            float map_absolute_z = robot_z - 0.3f;//看不到的，假设该点地形与机器狗当前站立的基准面齐平
            // float map_absolute_z = robot_z - 0.318f;
            if (this->_current_map.isInside(position)) //position的点在世界范围内部
            {
                try {
                    float temp_z = this->_current_map.atPosition(layer, position);
                    //取物理坐标周围的4个0.05栅格进行双线性插值（平滑）
                    // float temp_z = this->_current_map.atPosition(layer, position, grid_map::InterpolationMethods::INTER_LINEAR);
                    if (!std::isnan(temp_z) && !std::isinf(temp_z)) {
                        map_absolute_z = temp_z; // 把真实地形高度传给外层变量
                    }
                } catch (...) {
                }
            }

            // if (_filtered_robot_z == 0.0f) _filtered_robot_z = robot_z; // 初始化
            // _filtered_robot_z = 0.9f * _filtered_robot_z + 0.1f * robot_z;
            // // 计算高度差时，使用滤波后的 Z：
            // float height_diff = _filtered_robot_z - 0.3f - map_absolute_z;

            float height_diff = robot_z - 0.3f - map_absolute_z; // base_height_target = 0.3
            // float height_diff = robot_z - 0.318f - map_absolute_z; // base_height_target = 0.3这个是legged_gym中的 仿真中的trunk身高为0.318
            float clipped_height = std::max(-1.0f, std::min(1.0f, height_diff));
            relative_height = clipped_height * height_measurements;
            this->_scan_data.push_back(relative_height);
            // ================= 3. 将验证点加入 Marker =================
            // 为了验证采样位置和读取到的地形高度是否正确，我们将球画在真实的地形绝对高度上
            geometry_msgs::Point p;
            p.x = global_x;
            p.y = global_y;
            p.z = map_absolute_z; // 直接使用查表得到的绝对高度
            points_marker.points.push_back(p);
        }
    }
    // ================= 4. 循环结束后发布 Marker =================
    static int vis_counter = 0;
    if (vis_counter++ % 10 == 0) {
        this->sampled_points_pub_.publish(points_marker);
    }
}


void State_VRL::_init_buffers()
{
    this->_dYawCmdPast=0.0;

    this->_action = torch::zeros({1, 12});
    this->_observation = torch::zeros({1, this->_num_obs});
    this->_gravity_vec = torch::tensor({{0.0, 0.0, -1.0}});
    this->_obs_buffer_tensor = torch::zeros({1, this->_num_obs_history*this->_num_obs});
}

void State_VRL::exit()
{
    this->_percent_1 = 0;
    _threadRunning = false;//不进行循环
    if(_inferenceThread.joinable())//判断循环是否还在执行
        _inferenceThread.join();//等待循环结束
}

void State_VRL::_loadPolicy()//加载JIT模型
{
    //解决超时问题
    torch::set_num_threads(1);
    torch::set_num_interop_threads(1);

    this->_body_module = torch::jit::load(body_path);
    this->_adapt_module = torch::jit::load(adaption_module_path);
}

torch::Tensor State_VRL::QuatRotateInverse(torch::Tensor q, torch::Tensor v)
{
    c10::IntArrayRef shape = q.sizes();
    torch::Tensor q_w = q.index({torch::indexing::Slice(), -1});
    torch::Tensor q_vec = q.index({torch::indexing::Slice(), torch::indexing::Slice(0, 3)});
    torch::Tensor a = v * (2.0 * torch::pow(q_w, 2) - 1.0).unsqueeze(-1);
    torch::Tensor b = torch::cross(q_vec, v, /*dim=*/-1) * q_w.unsqueeze(-1) * 2.0;
    torch::Tensor c = q_vec * torch::bmm(q_vec.view({shape[0], 1, 3}), v.view({shape[0], 3, 1})).squeeze(-1) * 2.0;
    return a - b + c;
}

void State_VRL::_getUserCmd(){
#ifdef USE_XBOX_JOYSTICK
    //1.平移速度
    _vCmdBody(0) = 2.5 * invNormalize(_userValue.ly,_vxLim(0), _vxLim(1));
    _vCmdBody(1) = -2 * invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;
    //2.旋转速度 Yaw
    _dYawCmd = -10 * invNormalize(_userValue.rx,_wyawLim(0),_wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1 - 0.9) * _dYawCmd; // 低通滤波
    _dYawCmdPast = _dYawCmd;
    //* 3. 新增：计算目标 Heading (对角速度进行积分) */
    // 控制循环频率是 50Hz (dt = 0.02秒)
    float dt = 0.02; 
    this->_targetHeading += _dYawCmd * dt;

    // 限制在 [-pi, pi] 之间，防止数值溢出
    if (this->_targetHeading > M_PI) {
        this->_targetHeading -= 2 * M_PI;
    } else if (this->_targetHeading < -M_PI) {
        this->_targetHeading += 2 * M_PI;
    }
    //在legged_gym中这里的是把heading使用P控制器转化为角速度
    //获取当前狗的真实Yaw角 - 从四元数计算
    // float current_yaw = _lowState->imu.rpy[2]; // 直接取索引 2，即 Yaw 底层是四元数
    // 提取四元数 (w, x, y, z)
    float w = _lowState->imu.quaternion[0];
    float x = _lowState->imu.quaternion[1];
    float y = _lowState->imu.quaternion[2];
    float z = _lowState->imu.quaternion[3];

    // 计算 Yaw 角 (Z轴旋转)
    float current_yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    //计算heading误差
    float heading_error = this->_targetHeading - current_yaw;
    // c. 误差也必须限制在 [-pi, pi] 之间 (这是为了让狗懂得"走捷径"转头)
    if (heading_error > M_PI) {
        heading_error -= 2 * M_PI;
    } else if (heading_error < -M_PI) {
        heading_error += 2 * M_PI;
    }
    // d. 乘以 P 控制器系数 (对应 Python 里的: 0.5 * wrap_to_pi(commands[:, 3] - heading))
    float computed_yaw_vel = 0.5 * heading_error;
    
    // e. 限幅到 [-1.0, 1.0] (对应 Python 里的: torch.clip(..., -1., 1.))
    if (computed_yaw_vel > 1.0) {
        computed_yaw_vel = 1.0;
    } else if (computed_yaw_vel < -1.0) {
        computed_yaw_vel = -1.0;
    }
    _dYawCmd = computed_yaw_vel;
#else
    // 键盘控制模式 (VRL)
    _vCmdBody(0) = 2.5 * invNormalize(_userValue.ly, _vxLim(0), _vxLim(1));
    _vCmdBody(1) = -2.0 * invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0;

    // if (_vCmdBody(0) > 0.5) _vCmdBody(0) = 0.5;
    // if (_vCmdBody(0) < -0.5) _vCmdBody(0) = -0.5;
    _dYawCmd = -10.0 * invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9 * _dYawCmdPast + (1 - 0.9) * _dYawCmd; // 低通滤波
    _dYawCmdPast = _dYawCmd;
#endif
}

void State_VRL::_observations_compute()
{
    //使用网络自己估计线速度
    // //1.机体线速度(3维)
    // torch::Tensor body_lin_vel = torch::tensor({_est->getVelocity()[0],_est->getVelocity()[1], _est->getVelocity()[2]})
    // body_lin_vel = body_lin_vel * this->scale_lin_vel;

    // 2. 机体角速度 (3维)
    torch::Tensor body_ang_vel = torch::tensor({_lowState->imu.gyroscope[0], _lowState->imu.gyroscope[1], _lowState->imu.gyroscope[2]});
    body_ang_vel = body_ang_vel * this->scale_ang_vel;

    // 3. 投影重力 (3维)
    torch::Tensor base_quat = torch::tensor({{_lowState->imu.quaternion[1], _lowState->imu.quaternion[2], _lowState->imu.quaternion[3], _lowState->imu.quaternion[0]}});
    torch::Tensor projected_gravity = QuatRotateInverse(base_quat, this->_gravity_vec);
    // 先取第0行，再取第2列，最后转成 C++ 的 float 才能安全打印
    // std::cout << "Gravity Z: " << projected_gravity[0][2].item<float>() << std::endl;

    // 4.命令观测(3维)
    _userValue = _lowState->userValue;  // 获取用户输入
    _getUserCmd();  // 解析用户输入命令，更新速度命令
    // torch::Tensor commands = torch::tensor({_vCmdBody(0), _vCmdBody(1), 0.0});
    torch::Tensor commands = torch::tensor({_vCmdBody(0), _vCmdBody(1), _dYawCmd});

    // // 5. 关节相对位置 (12维)
    // // 根据URDF文件里的电机英文的名字顺序->宇树狗的电机FR、FL、RR、RL
    // // 网络的电机输出顺序是[FL, FR, RL, RR]  宇树电机的顺序是FR、FL、RR、RL
    // torch::Tensor dof_pos_tensor = torch::tensor({
    //     _lowState->motorState[3].q - this->_default_dof_pos[3], _lowState->motorState[4].q - this->_default_dof_pos[4], _lowState->motorState[5].q - this->_default_dof_pos[5],
    //     _lowState->motorState[0].q - this->_default_dof_pos[0], _lowState->motorState[1].q - this->_default_dof_pos[1], _lowState->motorState[2].q - this->_default_dof_pos[2],
    //     _lowState->motorState[9].q - this->_default_dof_pos[9], _lowState->motorState[10].q - this->_default_dof_pos[10], _lowState->motorState[11].q - this->_default_dof_pos[11],
    //     _lowState->motorState[6].q - this->_default_dof_pos[6], _lowState->motorState[7].q - this->_default_dof_pos[7], _lowState->motorState[8].q - this->_default_dof_pos[8]
    // });
    // dof_pos_tensor = dof_pos_tensor * this->scale_dof_pos;

    // // 6. 关节角速度 (12维) - 排序同上
    // torch::Tensor dof_vel_tensor = torch::tensor({
    //     _lowState->motorState[3].dq, _lowState->motorState[4].dq, _lowState->motorState[5].dq,
    //     _lowState->motorState[0].dq, _lowState->motorState[1].dq, _lowState->motorState[2].dq,
    //     _lowState->motorState[9].dq, _lowState->motorState[10].dq, _lowState->motorState[11].dq,
    //     _lowState->motorState[6].dq, _lowState->motorState[7].dq, _lowState->motorState[8].dq
    // });
    // dof_vel_tensor = dof_vel_tensor * this->scale_dof_vel;
    // 5. 关节相对位置 (12维)
    torch::Tensor dof_pos_tensor = torch::tensor({
        // FR (右前)
        _lowState->motorState[0].q - this->_default_dof_pos[0], 
        _lowState->motorState[1].q - this->_default_dof_pos[1], 
        _lowState->motorState[2].q - this->_default_dof_pos[2],
        // FL (左前)
        _lowState->motorState[3].q - this->_default_dof_pos[3], 
        _lowState->motorState[4].q - this->_default_dof_pos[4], 
        _lowState->motorState[5].q - this->_default_dof_pos[5],
        // RR (右后)
        _lowState->motorState[6].q - this->_default_dof_pos[6], 
        _lowState->motorState[7].q - this->_default_dof_pos[7], 
        _lowState->motorState[8].q - this->_default_dof_pos[8],
        // RL (左后)
        _lowState->motorState[9].q - this->_default_dof_pos[9], 
        _lowState->motorState[10].q - this->_default_dof_pos[10], 
        _lowState->motorState[11].q - this->_default_dof_pos[11]
    });
    dof_pos_tensor = dof_pos_tensor * this->scale_dof_pos;

    // 6. 关节角速度 (12维)
    torch::Tensor dof_vel_tensor = torch::tensor({
        // FR, FL, RR, RL
        _lowState->motorState[0].dq, _lowState->motorState[1].dq, _lowState->motorState[2].dq,
        _lowState->motorState[3].dq, _lowState->motorState[4].dq, _lowState->motorState[5].dq,
        _lowState->motorState[6].dq, _lowState->motorState[7].dq, _lowState->motorState[8].dq,
        _lowState->motorState[9].dq, _lowState->motorState[10].dq, _lowState->motorState[11].dq
    });
    dof_vel_tensor = dof_vel_tensor * this->scale_dof_vel;

    // 7. 组装最终观测张量 (严格按照 Python 训练时的拼接顺序！)
    // 总计: 3 + 3 + 3 + 3 + 12 + 12 + 12 = 48  - 3 = 45
    //_num_obs[线速度3 角速度3 重力加速度3 指令3  关键相对角度12 角速度12 历史观测12]
    auto current_obs = torch::cat({
        body_ang_vel.reshape({1, 3}),      
        projected_gravity.reshape({1, 3}), 
        commands.reshape({1, 3}) * scale_commands.reshape({1, 3}),          
        dof_pos_tensor.reshape({1, 12}),    
        dof_vel_tensor.reshape({1, 12}),    
        _last_action.reshape({1, 12})       
    }, -1); // 在第1维(列)拼接，结果为 [1, 45]

    // 8. 赋值并进行限幅处理
    this->_observation = torch::clamp(current_obs, -clip_observations, clip_observations);

    //V0
    // _last_action = _action.clone();
}

// 每次更新时，移除最旧的 _num_obs 个元素，并在前面追加新的观测数据
void State_VRL::_obs_buffer_update()
{
    _observations_compute();// 获取新观测数据并存储在 this->_observation 中
    //this->_obs_buffer_tensor = torch::cat({ 历史保留数据, 最新观测数据 }, -1); -1代表最后一个维度
    //this->_obs_buffer_tensor.narrow(1, this->_num_obs, (this->_num_obs_history*this->_num_obs-this->_num_obs))
    //narrow(dim,start,length) 砍掉最老的一帧 保留剩下的
    this->_obs_buffer_tensor = torch::cat({this->_obs_buffer_tensor.narrow(1, this->_num_obs, (this->_num_obs_history*this->_num_obs-this->_num_obs))
                                        ,this->_observation.view({1, this->_num_obs})},-1);
}

void State_VRL::_action_compute()
{
    //以50hz调用rl_policy来生成关节角度
    torch::NoGradGuard no_grad;
    torch::Tensor latent = _adapt_module.forward({this->_obs_buffer_tensor}).toTensor();
    torch::Tensor scan_tensor = torch::from_blob(this->_scan_data.data(), {1, 187}, torch::kFloat32).clone();
    torch::Tensor combined_input = torch::cat({this->_obs_buffer_tensor, latent}, -1); // 450 + 19 = 469维
    this->_action = _body_module.forward({combined_input, scan_tensor}).toTensor();

    //V0
    _last_action = _action.clone();

    torch::Tensor actions_scaled = this->_action * this->action_scale;
    int indices[] = {0, 3, 6, 9};
    for (int i : indices)
        actions_scaled[0][i] *= this->hip_scale_reduction; // 髋关节模型顺序

    for(int i=0; i<12; i++)
    {
        //网络的电机输出顺序是[FL, FR, RL, RR]  宇树电机的顺序是FR、FL、RR、RL
        // this->_joint_q[i] = actions_scaled[0][this->dof_mapping[i]].item<double>();
        this->_joint_q[i] = actions_scaled[0][i].item<double>();
        this->_joint_q[i] += this->_default_dof_pos[i];
    }
}

void State_VRL::_inferenceLoop()
{
    while (_threadRunning)
    {
        _start_RL_Time = getSystemTime();//获取开始执行的时间
        _obs_buffer_update();
        updateScanData();

        {
            // 作用域界定，用于控制下面第一行的生命周期，锁会在之后由析构函数释放
            std::lock_guard<std::mutex> lock(_mutex);
            _action_compute(); // 执行模型推理
        }
        _inferenceReady = true;
        this->_percent_1 = 0;
        absoluteWait(_start_RL_Time, (long long)(this->_RL_dt * 1000000));  // 绝对等待，保证控制周期，即线程频率50hz
    }
    
}

void State_VRL::enter()//进入下一个状态
{
    for(int i = 0;i < 12;i++)
    {
        //模式10代表闭环伺服控制模式（PMSM closed-loop control），也就是允许用户自定义发送位置、速度、力矩和 PD 参数的模式。
        _lowCmd->motorCmd[i].mode = 10;//设置电机的工作模式
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;  //把当前角度作为期望角度 原地锁死
        _lowCmd->motorCmd[i].dq = 0;//电机的期望速度为0
        _lowCmd->motorCmd[i].Kp = this->Kp;//设置PD控制的刚度
        _lowCmd->motorCmd[i].Kd = this->Kd;//设置PD控制的阻尼
        _lowCmd->motorCmd[i].tau = 0;//前馈力矩为0

        // 设置进入状态机时的初始关节角度为目标角度
        this->_targetPos_rl[i] = this->_default_dof_pos[i];  // 初始pd调整至默认关节位置
        this->_last_targetPos_rl[i] = _lowState->motorState[i].q;
        this->_joint_q[i] = this->_default_dof_pos[i];
    }

    _init_buffers(); // 初始化参数
    _loadPolicy();
    //初始化_obs_buffer_tensor
    this->_obs_buffer_tensor = torch::zeros({1,this->_num_obs_history * this->_num_obs});
    this->_action = torch::zeros({1,12});
    for (int i = 0; i < this->_num_obs_history; ++i) {
        _obs_buffer_update(); // 更新初始的观测buffer
    }

    std::cout << "[VRL] Starting inference thread..." << std::endl;
    // 启动线程
    _threadRunning = true;  // 线程循环启动
    _inferenceReady = false; // 推理完成标志位
    if (!_inferenceThread.joinable()) {  // joinable检查线程是否在执行中，执行中会返回true。
        _inferenceThread = std::thread(&State_VRL::_inferenceLoop, this);  // 创建线程
    }
}

void State_VRL::run()
{
    if (_inferenceReady) {
        std::lock_guard<std::mutex> lock(_mutex);

        // 🌟 修复关键：只在拿到新动作时，把上一次的目标点作为这一次插值的【固定起点】
        for (int j = 0; j < 12; j++) {
            this->_last_targetPos_rl[j] = this->_targetPos_rl[j];
        }

        // 获取 RL 网络输出的最新目标点
        memcpy(this->_targetPos_rl, this->_joint_q, sizeof(this->_joint_q));

        this->_percent_1 = 0.0f; // 重置插值进度
        _inferenceReady = false;
    }

    // ================== 2. 计算当前控制周期的插值进度 ==================
    // 假设 RL 是 50Hz，控制是 500Hz
    this->_percent_1 += 1.0f / this->_duration_1;
    this->_percent_1 = this->_percent_1 > 1.0f ? 1.0f : this->_percent_1;

    // ================== 3. 完美的线性插值发送 ==================
    for (int j = 0; j < 12; j++)
    {
        _lowCmd->motorCmd[j].mode = 10;

        // 真正的线性插值：起点恒定，终点恒定，只有 percent 在均匀变化
        float smoothed_q = (1.0f - this->_percent_1) * this->_last_targetPos_rl[j] +
                           this->_percent_1 * this->_targetPos_rl[j];

        _lowCmd->motorCmd[j].q = smoothed_q;
        _lowCmd->motorCmd[j].dq = 0;
        _lowCmd->motorCmd[j].Kp = this->Kp;
        _lowCmd->motorCmd[j].Kd = this->Kd;
        _lowCmd->motorCmd[j].tau = 0;

        // this->_last_targetPos_rl[j] = _targetPos_rl[j];

    }
}

FSMStateName State_VRL::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B)  // 接收到L2_B则切换到passive状态
    {
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){  // 接收到L2_A则进入fixstand状态
        return FSMStateName::FIXEDSTAND;
    }
    else{  // 否则保持RL
        return FSMStateName::VRL;
    }
}