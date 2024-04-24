# UAV guided landing
基于地面视觉的四旋翼无人机引导降落仿真
<br/><br/>

## 环境
仅为推荐环境，非必须
- Ubuntu 18.04
- ROS Melodic
<br/><br/>

## 概述
- aruco_ros: 相对位姿估计
- detector: 地面端程序
    - detector/config: bool参数(用LMPC还是NMPC、是否记录数据、是否有延迟)、LMPC和NMPC的权重、无人机悬停油门
    - detector/src/simulation_detector.cpp: 地面端主程序(数据记录、主循环定时器、降落判断)
    - detector/src/mpc.cpp: MPC初始化、微分方程、优化问题构建、约束、优化问题求解 
- model_editor_models: 仿真模型(搭载相机的地面平台、底部装有AprilTag的无人机)
- simulation: 仿真相关
- uav_control: 无人机端程序
<br/><br/>

## 依赖
1. [Prometheus](https://github.com/amov-lab/Prometheus "https://github.com/amov-lab/Prometheus")

    官方wiki：[Prometheus安装及编译](https://github.com/amov-lab/Prometheus/wiki/%E5%AE%89%E8%A3%85%E5%8F%8A%E7%BC%96%E8%AF%91 "https://github.com/amov-lab/Prometheus/wiki/%E5%AE%89%E8%A3%85%E5%8F%8A%E7%BC%96%E8%AF%91")

    其他教程：[Prometheus踩坑记录（仿真+实机)](https://zhuanlan.zhihu.com/p/393653110 "https://zhuanlan.zhihu.com/p/393653110")

    **！！！注意**：Prometheus依赖项中除ROS外，其他模块的依赖项均不需要安装。Prometheus安装完成后，编译这一步只需要编译control模块。
    ``` bash
    ./compile_control.sh
    ```

2. [casadi](https://github.com/casadi/casadi "https://github.com/casadi/casadi")

    官方wiki：[InstallationLinux](https://github.com/casadi/casadi/wiki/InstallationLinux "https://github.com/casadi/casadi/wiki/InstallationLinux")

    ``` bash
    git clone https://github.com/casadi/casadi.git
    cd casadi
    mkdir build
    cd build
    cmake -DWITH_IPOPT=ON -DWITH_OSQP=ON -DWITH_QPOASES=ON ..
    make
    sudo make install
    ```
<br/><br/>

## 运行
step 1: 进入到simulation/shell目录下，打开终端
``` bash
    cd ~/simulation/shell
```  
step 2: 根据要进行的实验，执行对应的```.sh```文件

**！！！注意**: 不同实验的主要差别是地面平台的位置、运动方式以及MPC权重参数，程序本身逻辑基本相同。 
1. 地面平台静止降落(无延迟)
    ``` bash
    ./auto_landing.sh 
    ```
    执行上面的```.sh```文件后会打开若干个终端，正常情况下(无红色报错)，在从右向左第2个终端窗口内应当显示：
    ```
    Input 1 to switch to offboard mode and arm the drone.
    ```
    键盘输入```1```并```Enter```，无人机解锁，螺旋桨开始转动，此时该窗口显示：
    ```
    Input 1 to takeoff.
    ```
    键盘输入```1```并```Enter```，无人机起飞，仿真实验开始。


2. 地面平台直线运动降落(无延迟)
    
    ``` bash
    ./auto_move_landing_straight.sh 
    ```
    在从右向左第2个终端窗口:
    ``` bash
    Input 1 to switch to offboard mode and arm the drone.
    1   #键盘输入并回车
    Input 1 to takeoff.
    1   #键盘输入并回车
    ```
    此时无人机应起飞并到达预设位置保持悬停，然后```Ctrl+Shift+T```或```Ctrl+Alt+T```另开一个终端，输入：
    ``` bash
    rostopic pub /moving_platform/wheelcontrol/cmd_vel geometry_msgs/Twist -r 1 [0.5,0.0,0.0] [0.0,0.0,0.0]
    ```
    此时地面平台应当做直线运动，从无人机下方经过。

3. 地面平台曲线运动降落(无延迟)
    ``` bash
    ./auto_move_landing_curve.sh 
    ```
    在从右向左第2个终端窗口:
    ``` bash
    Input 1 to switch to offboard mode and arm the drone.
    1   #键盘输入并回车
    Input 1 to takeoff.
    1   #键盘输入并回车
    ```
    此时无人机应起飞并到达预设位置保持悬停，然后```Ctrl+Shift+T```或```Ctrl+Alt+T```另开一个终端，输入：
    ``` bash
    rostopic pub /moving_platform/wheelcontrol/cmd_vel geometry_msgs/Twist -r 1 [0.5,0.0,0.0] [0.0,0.0,0.4]
    ```
    此时地面平台应当做曲线运动，从无人机下方经过。
<br/><br/>

## 参考
[1] [CoNi-MPC](https://github.com/fast-fire/CoNi-MPC "https://github.com/fast-fire/CoNi-MPC")