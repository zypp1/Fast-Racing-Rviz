# Fast-Racing-Rviz 

时间： 2024 年 12 月 10 日

论文：Fast-Racing: An Open-source Strong Baseline for SE(3) Planning in Autonomous Drone Racing

Github地址：

（1） https://github.com/ZJU-FAST-Lab/Fast-Racing 

（2） https://github.com/Dwl2021/Fast-Racing.git 

> 说明：由于 Airsim 启动网络问题，此代码学习了上述仓库（2）的方法，将仿真环境移植至 rviz 。
非常感谢作者（1）（2）为社区的贡献，本文中的一些具体修改细节均来源于（2），为便于查看，记录于本仓库，如有侵权，请告知删除。注意：文中所提及的代码行号均为原仓库中的代码行号。



## 1 参数说明

（1）规划器是否控制航向角 Yaw 设置

在文件 Fast-Racing/src/utils/traj_server/src/traj_server.cpp 中第 474、475 行处，通过注释不同的代码来进行选择

```cpp
_cmd.yaw = atan2(_cmd.velocity.y, _cmd.velocity.x);   // 使用速度方向作为航向角
//  _cmd.yaw = hover_yaw;                             // 使用固定的航向角
```

（2）rviz 仿真时无人机初始航向角的设置

这里涉及 Quadrator.cpp 和 traj_server.cpp 两个文件。其中， Quadrator.cpp 是无人机的动力学仿真模型文件，traj_server.cpp 是最终给控制器发送期望状态的文件。**注意：traj_server.cpp 中悬停航向 hover_yaw 的数值会覆盖 Quadrator.cpp 中的航向设置。**

Quadrator.cpp 文件中第 33 -36行即为无人机初始状态的设置。一般在 launch 文件中仅仅对初始位置进行了设置，该文件的 219-221 行即为从 launch 文件获取初始位置的代码。在程序中航向默认设置为正北，速度、角速度、电机转速均为 0。通过改写代码，对姿态矩阵赋值，可修改默认航向。由于无人机动力学的关系，若修改了初始的水平姿态，无人机会飞离设置的初始位置后重新恢复成水平状态。

```cpp
state_.x = Eigen::Vector3d::Zero();
// state_.x << 40.0, -60.0, 10.0;
state_.v         = Eigen::Vector3d::Zero();
state_.R         = Eigen::Matrix3d::Identity();  //默认姿态，航向正北
//Eigen::Matrix3d matrix_tmp;                    //自定义姿态，航向正东
//matrix_tmp << 0,-1, 0,
//              1, 0, 0,
//              0, 0, 1;
//state_.R         = matrix_tmp;
state_.omega     = Eigen::Vector3d::Zero();
state_.motor_rpm = Eigen::Array4d::Zero();
```

traj_server.cpp 文件中获取从 Quadrator.cpp 中订阅的 odom 信息后对无人机进行控制指令的发布。其中，通过在 24 行定义的变量 `hover_yaw` 可对初始悬停时的无人机航向进行控制，但是该参数会覆盖 Quadrator.cpp 中的航向设置。如果不启动 traj_server 节点，那无人机的航向就是 Quadrator.cpp 中定义的航向，反之 hover_yaw 将覆盖 Quadrator.cpp 中定义的航向。

```cpp
#define hover_yaw PI/2   //第24行
```

## 2 文件修改概要

（1）修改启动文件 zhangjiajie.launch 替换为自己修改后的 run_in_sim.launch

涉及文件：`zhangjiajie.launch`

（2）修改数据发布的参考坐标系，由 `/world_enu` 或 `/world` 修改为 `world`。

涉及文件：`zhangjiajie.launch`、`zhangjiajie_params.yaml`、`ctrl_md.launch`、`map_util.h`、`jps_planner.cpp`、`odom_visualization.cpp(两个都要)`、`waypoint_generator.cpp`、`OctomapServer.h`、`traj_server.cpp`

（3）修改路径点和终点坐标的获取方式，移除 airsim 部分。

涉及文件：`se3_node_cpu.cpp / se3_node_gpu.cpp`

（4）将地图部分由 AirSim 中的地图话题替换成自己的地图仿真节点发布的地图话题。将 `/airsim_global_map` 修改为 `/map_generator/global_cloud`。

涉及文件：`map_util.h`、`run_in_sim.launch` 

（5）修改里程计的订阅话题，将 `/airsim_node/drone_1/odom_local_enu` 修改为仿真节点发布的 odom 话题 `/visual_slam/odom`。

涉及文件：`run_in_sim.launch` 、`traj_server.launch`、`ctrl_md.launch`

（6）修改控制命令发布话题

涉及文件：`ctrl_md.launch`、`traj_server.launch`

（7）修改 imu 订阅话题

涉及文件：`ctrl_md.launch`

（8）选择是否可视化无人机 mesh

涉及文件：`MinCoPlan_CPU.cpp / MinCoPlan_GPU.cpp`

（9）修改地图原点的设置

涉及文件：`map_util.h`

一些暂时在 rviz 仿真中没有涉及到的文件暂时未做修改，以后有需要再修改并对文档进行补充。


## 3 文件修改详细说明

1. 文件 zhangjiajie.launch 替换为自己修改后的 run_in_sim.launch
路径：Fast-Racing/src/plan_manage/launch/zhangjiajie.launch

> 该文件是启动的核心程序之一，主要的修改包括：移除 airsim 仿真相关节点，替换 PointCloud_in 话题和 odom 话题 remap 的话题名称。修改文件主要参照 ego_planner 项目仿真的启动文件。

需要将 zhangjiajie.launch 替换为自己修改后的 run_in_sim.launch，并添加由 ego_planner 项目复制过来的 simulator.xml 文件。

该 launch 文件启动的节点主要包括 7 个节点：

> 前 4个节点与 rviz 仿真相关，定义在 simulator.xml 文件中，通过 `include` 标签被引入到 run_in_sim.launch 文件中使用

- map_generator
- so3_quadrotor_simulator
- so3_control
- odom_visualization

- traj_server
- waypoint_generator
- se3_node

将 se3_node 节点中 `~PointCloud_in` 、`~odom` 以及 `~world_frame_id` 分别修改为

```xml
<!-- 修改~PointCloud_in-->
<remap from="~PointCloud_in"    to="/map_generator/global_cloud"/>

<!--修改~odom-->
<remap from="~odom"             to="$(arg odom_topic)"/>

<!--修改~world_frame_id-->
<param name="world_frame_id"    value="world" type="string"/>
```

2. 文件 zhangjiajie_params.yaml
路径：Fast-Racing/src/plan_manage/misc/zhangjiajie_params.yaml

```yaml
# 第1行，修改订阅的地图话题为
MapTopic:                   '/map_generator/global_cloud'

# 第3行，修改参考坐标系为
OdomFrame:                  'world'
```

3. 文件 se3_node_cpu.cpp / se3_node_gpu.cpp
路径：src/Fast-Racing/src/plan_manage/src/se 3_node_cpu.cpp (se 3_node_gpu.cpp)

> 这部分需要修改路径点 gate_list 和目标点 target_pt 的获取方式，。其原程序中是从 Airsim 中获取每个门框的四个角点坐标并求和取平均，以得到每个门框的中心点坐标。我们可将这部分代码移除，通过直接给出路径点和终点坐标的方式，来为路径点 gate_list 和目标点 target_pt 变量赋值。 

这里给出一组修改后的简单示例，以 se3_node_cpu.cpp 文件为例：

```cpp
// 将第30-62行注释，并添加如下代码
std::vector<string> gates_list;
int gate_num = 3;   // 经过的路径点数量
    
Eigen::Vector3d centrl0(20, 0, 1);
gate_list.push_back(centrl0);
Eigen::Vector3d centrl1(20, 20, 1);
gate_list.push_back(centrl1);
Eigen::Vector3d centrl2(-20, 20, 1);
gate_list.push_back(centrl2);

// 将第64-79行注释，并添加如下代码
target_pt = Eigen::Vector3d(-20, -20, 1);
std::cout<<"goal: "<<target_pt.transpose()<<std::endl;
```

4. 文件 ctrl_md.launch
路径：Fast-Racing/src/Ctrl/launch/ctrl_md.launch
采用 gazebo 仿真或实机部署时需要修改该文件，仅用 rviz 仿真用不到该控制节点。

```xml
<!--第5行，根据实际情况来修改-->
<remap from="~odom" to="/mavros/local_position/odom" />

<!--第6行，根据实际情况来修改-->
<remap from="~cmd" to="/position_cmd" />

<!--第7行，根据实际情况来修改-->
<remap from="~imu" to="/mavros/imu/data" />
```

5. 文件 map_util.h
路径： Fast-Racing/src/path_searching/include/jps_collision/map_util.h

```cpp
// 第64行，修改参考坐标系为
nh.param("world_frame_id",world_frame_id,std::string("world"));

// 第67行，修改地图原点y坐标的设置为
origin_d_[1] = -map_size(1)/2;

// 第82行，修改路径前端搜索搜索算法订阅的地图消息名称
point_cloud_sub_ = nh.subscribe("/map_generator/global_cloud", 10, &MapUtil::GlobalMapBuild, this);
```

6. 文件 jps_planner.cpp
路径：Fast-Racing/src/path_searching/src/jps_planner/jps_planner.cpp

```cpp
// 第13行，修改参考坐标系为
nh.param("world_frame_id",world_frame_id,std::string("world"));
```

7. 文件 odom_visualization.cpp（由/world 修改为 world）
路径：Fast-Racing/src/plan_manage/misc/odom_visualization/src/odom_visualization.cpp

> 这个 odom 可视化的功能包，由于是在 plan_manage 功能包下的一个文件夹中，按照 ros 的规则，其是不会被编译的。

```cpp
// 第586、587行，修改参考坐标系为
n.param("base_frame", baseFrame, std::string("world"));
n.param("target_frame", target_frame, std::string("world"));
```

8. 文件 odom_visualization.cpp（由/world_enu 修改为 world），该文件与上面的不是同一个，在不同的文件夹下。
路径：Fast-Racing/src/utils/uav_simulator/Utils/odom_visualization/src/odom_visualization.cpp

>这个是实际起作用的 odom_visualization

```cpp
// 第84行，修改参考坐标系为
poseROS.header.frame_id = string("world");

// 第101行，修改参考坐标系为
velROS.header.frame_id = string("world");

// 第170行，修改参考坐标系为
covROS.header.frame_id = string("world");

// 第219行，修改参考坐标系为
covVelROS.header.frame_id = string("world");

// 第249行，修改参考坐标系为
trajROS.header.frame_id = string("world");

// 第290行，修改参考坐标系为
sensorROS.header.frame_id = string("world");

// 第384-387行，修改参考坐标系为，移除了坐标系前的符号 /
broadcaster->sendTransform(tf::StampedTransform(transform,   msg->header.stamp, string("world"),  string("base")));
broadcaster->sendTransform(tf::StampedTransform(transform45, msg->header.stamp, string("base"), string("laser")));
broadcaster->sendTransform(tf::StampedTransform(transform45, msg->header.stamp, string("base"), string("vision")));
broadcaster->sendTransform(tf::StampedTransform(transform90, msg->header.stamp, string("base"), string("height")));

// 第453行，修改参考坐标系为
n.param("frame_id",   _frame_id, string("world") );    
```

9. 文件 waypoint_generator.cpp
路径：Fast-Racing/src/utils/waypoint_generator/src/waypoint_generator.cpp

```cpp
// 第87行，修改参考坐标系为
waypoints.header.frame_id = std::string("world");

// 第101行，修改参考坐标系为
poseArray.header.frame_id = std::string("world");
```

10. 文件 MinCoPlan_CPU.cpp / MinCoPlan_GPU.cpp 
路径：Fast-Racing/src/plan_manage/src/MinCoPlan_CPU.cpp (MinCoPlan_GPU.cpp) 

```cpp
// 第141行或142行，可将注释取消，以便发布无人机连续的机体轨迹，用以显示轨迹上不同位置处无人机的姿态
visualization.visualizeQuadrotor(traj, 70);
```

11. 文件 OctomapServer.h
路径：Fast-Racing/src/octomap_server/include/octomap_server/OctomapServer.h

```cpp
// 第91行，修改参考坐标系为
OctomapServer(const ros::NodeHandle private_nh_ = ros::NodeHandle("~"),const ros::NodeHandle &nh_ = ros::NodeHandle(),const std::string frame_id =  "world");
```

12. 文件 traj_server.launch
路径：Fast-Racing/src/utils/traj_server/launch/traj_server.launch

```xml
<!--第3行，修改发布的控制命令话题为-->
<remap from="~/position_command" to="/position_cmd"/>

<!--第4行，修改订阅的odom话题为-->
<remap from="~/odometry"      to="/visual_slam/odom"/>
```

13. 文件 traj_server.cpp
路径：Fast-Racing/src/utils/traj_server/src/traj_server.cpp

```cpp
// 第227行，修改参考坐标系为
_vis_cmd.header.frame_id = "world";

// 第238行，修改参考坐标系为
_cmd.header.frame_id = "world";

// 第361行，修改参考坐标系为
if (_cmd.header.frame_id != "world"){
    _cmd.position = _odom.pose.pose.position;
}

// 第370行，修改参考坐标系为
_cmd.header.frame_id = "world";

// 第402行，修改参考坐标系为
_cmd.header.frame_id = "world";

// 第498行，修改参考坐标系为
_vis_vel.header.frame_id = "world";

// 第550行，修改参考坐标系为
_vis_acc.header.frame_id = "world";
```