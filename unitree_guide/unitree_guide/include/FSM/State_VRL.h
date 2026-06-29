#ifndef VRL_H
#define VRL_H

#include "FSM/FSMState.h"
#include <torch/torch.h>
#include <torch/script.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstring> // 对于memcpy
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
// TF2头文件
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class State_VRL : public FSMState
{
    public:
    // CtrlComponents *ctrlComp 就是你赋予这个 C++ 状态类“操纵物理世界”的唯一授权书！
        State_VRL(CtrlComponents *ctrlComp);
        ~State_VRL() {}
        void enter();
        void run();
        void exit();
        FSMStateName checkChange();
    
        void initRos(ros::NodeHandle& nh);
        void mapCallback(const grid_map_msgs::GridMapConstPtr& msg);
        void updateScanData();
    private:

        float _filtered_robot_z = 0.3f;

        ros::Publisher sampled_points_pub_;

        tf2_ros::Buffer _tf_buffer;               // 🌟 确保加上这一行
        tf2_ros::TransformListener* _tf_listener; // 🌟 确保加上这一行
        //可以使用这里的 也可以使用CmakeLists.txt中的set
        // const std::string adaption_module_path = "/home/rosxiaobai/unitree_rl_VRL/src/unitree_guide/unitree_guide/model/VRL/adapt_module.jit";
        // const std::string body_path = "/home/rosxiaobai/unitree_rl_VRL/src/unitree_guide/unitree_guide/model/VRL/body_vision_module.jit";
        // const std::string adaption_module_path = "/home/rosxiaobai/unitree_rl_VRL/src/unitree_guide/unitree_guide/model/VRL_DelayTime/adapt_module.jit";
        // const std::string body_path = "/home/rosxiaobai/unitree_rl_VRL/src/unitree_guide/unitree_guide/model/VRL_DelayTime/body_vision_module.jit";
        const std::string adaption_module_path = "/home/rosxiaobai/unitree_rl_VRL/src/unitree_guide/unitree_guide/model/VRL_Debug_Kp_40/adapt_module.jit";
        const std::string body_path = "/home/rosxiaobai/unitree_rl_VRL/src/unitree_guide/unitree_guide/model/VRL_Debug_Kp_40/body_vision_module.jit";
        // const std::string adaption_module_path = "/home/rosxiaobai/unitree_rl_VRL/src/unitree_guide/unitree_guide/model/VRL_transformer/adapt_module.jit";
        // const std::string body_path = "/home/rosxiaobai/unitree_rl_VRL/src/unitree_guide/unitree_guide/model/VRL_transformer/body_vision_module.jit";

        float _targetPos_rl[12];  // 下发给电机的目标关节角度
        float _last_targetPos_rl[12];  // 上一次下发给电机的目标关节角度
        const float _default_dof_pos[12] = {-0.1, 0.8, -1.5, // FR
                                        0.1, 0.8, -1.5, //FL
                                        -0.1, 1.0, -1.5, // RR
                                        0.1, 1.0, -1.5 // RL
                                        };

        // const float _duration_1 = 10;
        const float _duration_1 = 2.0;

        float _percent_1 = 0;

        // 多线程
        std::thread _inferenceThread; // 推理线程
        std::mutex _mutex;  // 互斥锁
        std::atomic<bool> _threadRunning {true};  // 线程循环启动标志位
        std::atomic<bool> _inferenceReady {false};  // 推理完成标志位
        bool _first_action_received = false;  // 首帧动作标志，Kp缓降用
        void _inferenceLoop();  // 模型推理循环函数，在一个独立的线程中执行
        long long _start_RL_Time;  // 记录线程启动时间
        const double _RL_dt = 0.02; // RL控制频率50hz

        // RL推理
        void _loadPolicy();
        void _observations_compute();
        void _obs_buffer_update();
        void _action_compute();
        
        torch::jit::script::Module _body_module;//运动控制皮层 接受信息输出Action
        torch::jit::script::Module _adapt_module;//adapt_module.pt 环境感知小脑
        
        // const float Kp = 20.0;
        // const float Kd = 0.5;
        const float Kp = 40.0;
        const float Kd = 1.0;
        //_num_obs[线速度3 角速度3 重力加速度3 指令3  关键相对角度12 角速度12 历史观测12]
        const int _num_obs = 45;//不要线速度
        const int _num_obs_history = 10;
        const float clip_observations = 100.0;  // 观测限幅
        const float clip_actions = 100.0;  // 动作限幅
        const float action_scale = 0.25;
        const float hip_scale_reduction = 0.5;// 臀关节进一步缩放

        const float scale_lin_vel = 2.0;
        const float scale_ang_vel = 0.25;
        torch::Tensor scale_commands = torch::tensor({scale_lin_vel, scale_lin_vel, scale_ang_vel});
        // num_commands = 4  # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading
        // torch::Tensor scale_commands = torch::tensor({scale_lin_vel, scale_lin_vel, scale_ang_vel, 1.0f});
        float scale_dof_pos = 1.0;  // 关节位置
        float scale_dof_vel = 0.05;  // 关节速度
        // 对应: {lin_vel_x, lin_vel_y, ang_vel_yaw, heading}
        //, torch::dtype(torch::kDouble)
        torch::Tensor _obs_buffer_tensor = torch::zeros({1, _num_obs_history*_num_obs});  // 观测缓存
        float _joint_q[12];  // RL推理出的关节角度
        torch::Device device = torch::kCUDA;  // 如需调用GPU

        Estimator *_est;  // 状态估计器
        QuadrupedRobot *_robModel;  // 机器人模型
        VecInt4 *_contact;  // 接触力，先用着，大概率有问题TODO：
        Vec3 _vCmdBody;   // 身体线速度
        double _dYawCmd;  // z轴角速度
        double _dYawCmdPast; // 上一个z轴角速度
        Vec2 _vxLim, _vyLim, _wyawLim;  // 速度限幅
        RotMat _B2G_RotMat, _G2B_RotMat;  // 旋转矩阵

        double _targetHeading;

        torch::Tensor _action = torch::zeros({1, 12});  // 动作
        torch::Tensor _last_action = torch::zeros({1, 12});  // 上一帧动作
        torch::Tensor _observation = torch::zeros({1, _num_obs});  // 观测
        torch::Tensor _gravity_vec; // 重力矩阵


        void _getUserCmd();  // 解析用户输入命令，更新_vCmdBody和_dYawCmd
        void _init_buffers();  // 参数初始化
        torch::Tensor QuatRotateInverse(torch::Tensor q, torch::Tensor v);
        const int dof_mapping[12] = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

        const int num_scan = 187;
        std::vector<float> _scan_data;
        std::mutex _map_mutex;
        ros::Subscriber _elevation_sub;
        grid_map::GridMap _current_map;
        bool _has_map;
        const float height_measurements = 5.0;//地形高度缩放

};
#endif