# Catkin Graph

## Overview
* **Package**. Catkin Package 是ROS应用程序代码的组织单元,包含：
    1. 程序库，可执行文件, 脚本等
    2. Manifest(`package.xml`)清单：定义Package间的依赖关系
    - `rospack <command> [package_name]`：获取package的相关信息, `roscd [package_name]`, `rosls [package_name]`
    - 通过`catkin_make`进行编译
* **Node**. Package中的可执行文件即为 Node, Node间通信：
    1. 一个节点发布`Message`到`Topic`, 另一个节点订阅`Topic`来获取`Message`
    2. 提供和使用Service
    - 需要依赖于客户端库(`ROS Client Libraries`)来使不同编程语言编写的节点可以互相通信。
        * `rospy` = python 客户端库
        * `roscpp` = c++ 客户端库
        * `rosjs` = javascripts客户端库
        * `rosjava` = java客户端库
    - 查看`node`: `rosnode list`, `rosnode info node_name`
    - 运行`package`中的节点: `rosrun [package_name] [node_name]`
    - 运行`launch`文件中的多个节点: `roslaunch [package] [filename.launch]`
* **Topics**. `Node`发布和订阅`Topic`来传递`Message`
    - 显示正在运行的`Node`和`Topic`：`apt install ros-noetic-rqt && rosrun rqt_graph rqt_graph`
    - `rostopic [command]`: 获取`Topic`相关信息
* **Service**. 由某个`Node`提供，其他`Node`发送请求(`request`)并获得响应(`response`)
    - `rosservice call [service] [args]`: 调用`service`
* **msg** 和 **srv**: 分别描述消息类型和一项服务
    - msg在package的msg目录下, 每行声明一个数据类型和变量名
    `geometry_msgs/Point32 position`
    - srv在package的srv目录下，分为请求和响应两部分，由`---`分隔, A为请求，Sum为响应
    `int64 A  
    ---
    int64 Sum`

## Catkin Package
Catkin package inside a catkin work space (each executable file in the package is a `Node`)
```
workspace_folder/        -- WORKSPACE工作空间
  src/                   -- SOURCE SPACE 源码目录
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

## Graph Notions
- `Nodes`:节点, 一个节点即为一个可执行文件，它可以通过ROS与其它节点进行通信。
- `Messages`:消息， 消息是一种ROS数据类型(i.e., `geometry_msgs/Twist`)，用于订阅或发布到一个话题。
- `Topics`:话题, 节点可以发布消息到话题，也可以订阅话题以接收消息。
- `Master`:节点管理器， ROS名称服务 (比如帮助节点找到彼此)。
- `rosout`: ROS中相当于`stdout`/`stderr`。
- `roscore`: 主机+ `rosout` + 参数服务器 (需要在允许所有ROS程序前运行，`roslaunch`会自动调用`roscore`)

