首先先使用没有视觉的网络 进行流程学习
然后出错误部分：
1.电机的顺序：需要关注在训练的时候，网络的输入的电机的顺序FR, FL, RR, RL
2.平滑部分:_duration_1=10 
原本_duration_1 = 700 而且 
this->_last_targetPos_rl[j] = _targetPos_rl[j];这样平滑部分没有起作用
3.就是第三个指令是角度还是角速度

然后添加摄像头  
当你启动 RViz 的 launch 时（先运行）：
它在后台启动了一个名为 /robot_state_publisher 的节点，负责为你计算机器人的骨架并广播给 RViz。
当你启动 Gazebo 的 launch 时（后运行）：
宇树原厂的 gazeboSim.launch 文件里面，也自带启动了一个名为 /robot_state_publisher 的节点（用来把 Gazebo 里真实机器人的状态变成坐标发布出来）。

添加模型的时候 注意Livox_Mid_gazebo_sensor这个宏 
这个宏中不仅写了雷达的物理位置，还把 <gazebo> 插件直接打包塞进这个宏里了
    <xacro:Livox_Mid_gazebo_sensor name="livox_link" visualize="True" />
