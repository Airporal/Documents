# ROS2 基础知识

> ROS2 是第二代机器人系统，相比ROS1，ROS2对python3更加友好，不需要主节点（roscore）来为各个节点建立通信，支持linux\win\mac\嵌入式RTOS平台，采用DDS通信，实时性、可靠性、连续性更强......

------

[TOC]



------

## 1.ROS2安装

根据[鱼香ROS](https://fishros.com/d2lros2/#/humble/chapt1/get_started/3.%E5%8A%A8%E6%89%8B%E5%AE%89%E8%A3%85ROS2)提供的一键安装脚本完成ROS2安装，注意版本选择。

```bash
wget http://fishros.com/install -O fishros && . fishros
```

## 2.ROS2使用

和ROS类似，ROS2的通信方式主要有话题（Topic)、服务（Service)、动作（Action)、参数（Param)；此外，还支持TF坐标管理服务、Rviz2数据显示平台、Gazebo物理仿真平台、Rqt集成的qt界面。

ROS2与ROS之间可以通过功能包ros1_bridge实现通信。

https://us.freecat.cloud/api/v1/client/subscribe?token=0b91d8dff499e590cc000b5b6b2c460a

### 2.1工作空间与编译

#### 2.1.1建立ROS2工作空间

工作空间就是一个包含src文件夹的文件夹。

```bash
mkdir -p name_ws/src
cd name_ws/src
```

和ROS不同的是不需要初始化。直接在src文件夹下建立自己的功能包。

功能包是ROS2直接调用的包，包内包含多个可执行文件（cpp\py)。

```bash
ros2 pkg create <package_name> --build-type {cmake,ament_cmake,ament_python} --dependencies <依赖名字>
```

使用该命令在**src文件夹**下建立一个功能包，名为package_name；

功能包的ament编译类型，python选择ament_python,cpp选择cmake；

将该功能包用到的依赖添加到**--dependencies后**,如rclpy\rclcpp等。

调用该命令后，会在src目录下新建**package_name文件夹**，并在该文件夹下生成setup.cfg、setup.py、test、package.xml、resource、package_name文件。

+ 新建py类型节点：

  建立功能包时使用ament_python编译方式，在功能包路径下生成setup.cfg、setup.py、package.xml、resource、package_name、test文件

  ![image-20240915104623911](https://cdn.jsdelivr.net/gh/Airporal/Pictures/img/202409151046948.png)

  在功能包目录下的功能包名的子目录（该目录下有__init__.py文件）下新建自己的py文件。

```bash
cd example_topic_rclpy/example_topic_rclpy
touch your.py
```

+ 新建cpp类型节点：

  建立功能包时使用ament_cmake编译方式，在功能包路径下生成package.xml、CmakeLists.txt、include、src文件。

  ![image-20240915104638846](https://cdn.jsdelivr.net/gh/Airporal/Pictures/img/202409151046881.png)

  在该目录下的src目录下新建自己的cpp文件。

```bash
cd src/test2/src
touch aa.cpp
touch bb.cpp
```

#### 2.1.2功能包的编译

使用colcon工具编译ros2功能包（其它工具也行）。

colcon工具使用：

```bash
# 安装colcon
sudo apt-get install python3-colcon-common-extensions
# 直接编译所有包
cd ws/
colcon build
# 只编译工作空间下的某个功能包
colcon build --packages-select 功能包名
# 建立软链接的形式构建install文件，此时修改src文件，install会同时修改，而不用编译
colcon build –symlink-install
```

编译后会在工作空间生成build(存放编译文件)、install(存放安装文件）、log目录(存放日志文件)。

+ cpp功能包编译

  修改CmakeLists.txt文件

```cmake
# 之后ros2调用该节点就是使用该可执行文件
add_executable(节点名 src/编写的cpp文件名)
ament_target_dependencies(节点名 rclcpp)
#
install(TARGETS
  节点名
  DESTINATION lib/${PROJECT_NAME}
)
```

​		建议编写cpp文件时，将新建的节点和文件名设置相同，以免遗忘。

编译：

```bash
cd ws/
colcon build --packages-select cpp功能包名
# 编译完更新环境变量
source install/setup.bash
```

+ python功能包编译

  修改setup.py文件

```python
    entry_points={
        'console_scripts': [
            "节点名a = example_topic_rclpy.topic_publisher_02:main",
        ],
    },
```

​		在该文件中，将**节点名**设置为**功能包.编写的py文件名：：main**的形式，表示在ros2运行该节点时，执行该py文件。建议节点名和py文件名一致。

​		python功能包无需编译即可运行：

```bash
cd ws/
source install/setup.bash
ros2 run 功能包 节点名
```

#### 2.1.3从源码安装功能包

可以通过apt命令下载功能包，下载的功能包会放在/opt/ros/foxy/bin/目录下，配置文件在/opt/ros/foxy/share/目录下。

```bash
sudo apt install ros-foxy-功能包
```

当功能包出现问题时，也可手动编译安装获取：

```bash
# 首先新建工作空间
mkdir -p ~/ament_ws/src
cd ament_ws/src
# 下载源码功能包，也可手动下载
git clone https-url 自定义的名字
# 编译安装
cd ament_ws
colcon build
```

#### 2.1.4 ros2 pkg命令

ros2 pkg中有许多关于功能包的命令：

```bash
# 列出所有包，没有找到则需要source
ros2 pkg list
# 查看某个包包含的所有可执行文件名字，运行时一般不直接写可执行文件而写节点调用
ros2 pkg executables package_name：
# 查看包的路径
ros2 pkg prefix package-name
# 列出包的具体信息
ros2 pkg xml package
```

#### 2.1.5 ros2 node命令

ros2 node 中有许多关于节点的命令。

```bash
# 查看节点列表
ros2 node list
# 查看某个节点具体信息
ros2 node info 节点名
# 重映射节点名
ros2 run <package_name> <executable_name> --ros-args –remap __node:=自定义的名字
```

### 2.2话题

编写前，修改vscode的环境配置，使得可以找到代码的头文件。

![image-20240915104650919](https://cdn.jsdelivr.net/gh/Airporal/Pictures/img/202409151046967.png)

> 话题是最基础的通信方式，一个节点发布话题**（发布者）**，其它节点可通过话题名和话题**消息接口**订阅该话题**（订阅者）**，以实现单向一对多通信。:newspaper:

#### 2.2.1 ros2 topic命令

ros2 topic包含了关于话题的命令。

```bash
# 查看所有话题
ros2 topic list -t # -t 显示话题消息接口类型
# 实时打印话题
ros2 topic echo 话题名
# 查看话题具体信息
ros2 topic info 话题名
# 查看接口类型的具体信息
ros2 topic interface show 消息接口类型
# 通过该话题发布数据
ros2 topic pub 话题名 消息类型 数据
```

使用pub发布话题时，格式为'参数名:  value'，要和消息接口完全一致。

#### 2.2.2 建立发布者

发布者建立步骤为：

1. 导入数据类型和需要的包|头文件

   + cpp类型，需要在cmakelist和packages.xml中添加消息类型的依赖:

     ```cmake
     # cmakelist消息接口
     find_package(std_msgs REQUIRED)
     ament_target_dependencies(topic_publisher_01 rclcpp std_msgs)
     ```

     ```xml
     # packages.xml消息依赖
     <depend>std_msgs</depend>
     ```

     在src下新建cpp文件，并编写cpp代码，首先导入头文件：

     ```c++
     #include "rclcpp/rclcpp.hpp"
     #include "std_msgs/msg/string.hpp"
     ```

   + python类型，直接在新建的py文件下导入

     ```python
     #!/usr/bin/python3
     import rclpy
     from rclpy.node import Node
     ```

     第一句声明使用的python解释器，可根据自己环境更改。

2. 创建节点

   + cpp类型，使用rclcpp::init(argc,argv)初始化节点类，并创建一个节点。

     ```c++
     rclcpp::init(argc,argv);
     auto node = std::make_shared<rclcpp::Node>("node_01");
     ```

     **auto**是c++11中引入的自动类型关键字，可以根据变量的初始表达式自动得到变量类型。

     **std::make_shared**用于动态分配内存并构造一个对象，返回一个指针指向该对象。可用来实例化一个类。

     ```c++
     std::make_shared<创建的对象的类型名>(args)
     ```

     总而言之，声明了一个rclcpp::node类型，名为node_01的节点，node是该节点的指针，可通过node访问该节点属性。

   + python类型

     使用rclpy.init(args=args)初始化，并创建一个节点对象实例。

     ```python
     #!/usr/bin/env python3
     import rclpy
     from rclpy.node import Node
     
     class class_name(Node):
     	pass
     def main(args=None):
         rclpy.init(args=args) # 初始化rclpy
         node = class_name("topic_publisher_02")  # 新建一个节点
         rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
         rclpy.shutdown() # 关闭rclpy
     
     ```

3. 创建发布者

   + cpp类型，需要声明发布者，声明一个定时器以定时发布话题。一般使用类来定义，将声明内容放在private部分进行，构造函数在public部分进行。

     ```c++
     // 声明发布者指针，<>部分为消息接口类型
     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_money;
     // 声明定时器指针，负责定时处理发布逻辑
     rclcpp::TimerBase::SharedPtr timer_;
     ```

     完成声明后，需要对变量进行赋值、定义：

     ```c++
     // 发布者定义
     pub_money = this->create_publisher<std_msgs::msg::String>("topic_name",10);
     // 定时器定义,传入参数为周期:500ms,回调函数
     timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&类名::回调函数名, this));
     ```

     发布者定义调用了指针this，使用继承自rclcpp::Node的方法，创建名为topic_name的话题，参数10为消息队列长度。

     定时器定义时使用了一个定时回调函数，首先使用**std::bind建**立了一个函数对象，传入参数为绑定的函数地址、调用者地址，传入的参数占位符。在建立回调函数时一般使用此方法，回调函数接受参数的个数由占位符的个数确定。

     ```c++
     // 由于需要再类的内部调用该函数，所以传入this，每次接收两个参数，则设置两个占位符
     std::bind(&fun,this,_1,_2)
     ```

   + python类型

     首先继承并初始化节点，然后建立发布者和定时器。

     ```python
     #!/usr/bin/env python3
     import rclpy
     from rclpy.node import Node
     from std_msgs.msg import String
     class NodePublisher02(Node):
         def __init__(self,name):
             super().__init__(name)
             # 打印日志输出
             self.get_logger().info("大家好，我是%s!" % name)
             self.command_publisher_ = self.create_publisher(String,"command", 10) 
             self.timer = self.create_timer(0.5, self.timer_callback)
         
         def timer_callback(self):
             pass
     ```

     和cpp的函数接口大致一样，但是回调函数只需直接调用即可。

4. 写发布逻辑

   + cpp 类型

     发布逻辑需要定时发送，一般可在定时器内完成，如果有其他循环结构，也可在那些部分编写。

     rclcpp的日志输出函数RCLCPP_INFO常用来做printf调试用。

     ```cpp
     # 相当于rosinfo，支持格式化输出
     RCLCPP_INFO(this->get_logger(), "输出消息");
     ```

     **完整发布者代码：**

     ```cpp
     #include "rclcpp/rclcpp.hpp"
     #include "std_msgs/msg/string.hpp"
     
     class class_Name : public rclcpp::Node
     {
     public:
         // 构造函数,有一个参数为节点名称
         TopicPublisher01(std::string name) : Node(name)
         {
             RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
             // 创建发布者
             command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
             // 创建定时器，500ms为周期，定时发布
             timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&class_Name::timer_callback, this));
         }
     
     private:
         void timer_callback()
         {
             // 创建消息
             std_msgs::msg::String message;
             message.data = "forward";
             // 日志打印
             RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
             // 发布消息
             command_publisher_->publish(message);
         }
         // 声名定时器指针
         rclcpp::TimerBase::SharedPtr timer_;
         // 声明话题发布者指针
         rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
     };
     
     int main(int argc,char ** argv)
     {
         //初始化节点
         rclcpp::init(argc,argv);
         //建立节点cnode
         auto node = std::make_shared<class_Name>("node_name");
         //运行节点，并检测输出信号
         rclcpp::spin(node);
         //停止节点运行
         rclcpp::shutdown();
         return 0;
     }
     ```

     发布消息前，需要实例化消息接口类型，并赋值。

   + python类型

     编写回调函数，并在回调函数中实例化消息接口类型、赋值，然后按照制定周期发送。

     完整代码：

     ```python
     #!/usr/bin/env python3
     import rclpy
     from rclpy.node import Node
     from std_msgs.msg import String
     class NodePublisher02(Node):
         def __init__(self,name):
             super().__init__(name)
             self.get_logger().info("大家好，我是%s!" % name)
             self.command_publisher_ = self.create_publisher(String,"command", 10) 
             self.timer = self.create_timer(0.5, self.timer_callback)
         
         def timer_callback(self):
             """
             定时器回调函数
             """
             msg = String()
             msg.data = 'backup'
             self.command_publisher_.publish(msg) 
             self.get_logger().info(f'发布了指令：{msg.data}')    #打印一下发布的数据
     def main(args=None):
         rclpy.init()
         node = NodePublisher02("node_name")
         rclpy.spin(node)
         rclpy.shutdown()
     ```

#### 2.2.3 建立订阅者

   建立订阅者步骤为：

1. 导入数据类型和需要的包|头文件

   + cpp类型

     导入需要订阅的话题的消息接口类型头文件。同2.2.1.1。

   + python类型

     导入需要订阅的话题的消息接口类型头文件。同2.2.1.1。

   2. 创建节点

   + cpp类型

     使用roscpp类建立一个ros2节点。同2.2.1.1

   + python类型

     使用rospy类建立一个ros2节点。同2.2.1.1

   3. 创建订阅者

   + cpp类型

     与发布者类型，使用以下命令建立订阅者：

     ```cpp
     private:
     	// 在此声明订阅者，参数为<消息接口类型>
     	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscribe_;
     public:
     	//在此赋值订阅者，参数为订阅的话题、消息队列长度、回调函数（接收一个参数，所有有一个占位符）
     	command_subscribe_ = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&TopicSubscribe01::command_callback, this, std::placeholders::_1));
     
     ```

   + python类型

     ```python
      self.command_subscribe_ = self.create_subscription(String,"command",self.command_callback,10)
     ```

   4. 订阅回调逻辑

   + cpp类型

     在private中建立回调函数逻辑，回调函数的参数一般设置为如下所示：

     ```cpp
         void command_callback(const std_msgs::msg::String::SharedPtr msg)
     ```

     使用->运算符来调用msg的内容。

     整体代码如下：

     ```cpp
     #include "rclcpp/rclcpp.hpp"
     #include "std_msgs/msg/string.hpp"
     
     class TopicSubscribe01 : public rclcpp::Node
     {
     public:
         TopicSubscribe01(std::string name) : Node(name)
         {
             RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
               // 创建一个订阅者订阅话题
             command_subscribe_ = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&TopicSubscribe01::command_callback, this, std::placeholders::_1));
         }
     
     private:
          // 声明一个订阅者
         rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscribe_;
          // 收到话题数据的回调函数
         void command_callback(const std_msgs::msg::String::SharedPtr msg)
         {
             double speed = 0.0f;
             if(msg->data == "forward")
             {
                 speed = 0.2f;
             }
             RCLCPP_INFO(this->get_logger(), "收到[%s]指令，发送速度 %f", msg->data.c_str(),speed);
         }
     };
     int main(int argc,char ** argv)
     {
         //初始化节点
         rclcpp::init(argc,argv);
         //建立节点cnode
         auto node = std::make_shared<TopicSubscribe01>("node_name");
         //运行节点，并检测输出信号
         rclcpp::spin(node);
         //停止节点运行
         rclcpp::shutdown();
         return 0;
     }
     ```

   + python类型

     向回调函数中传递参数msg，整体代码如下：

     ```python
     #!/usr/bin/env python3
     import rclpy
     from rclpy.node import Node
     from std_msgs.msg import String
     class NodeSubscribe02(Node):
         def __init__(self,name):
             super().__init__(name)
             self.get_logger().info("大家好，我是%s!" % name)
             # 创建订阅者
             self.command_subscribe_ = self.create_subscription(String,"command",self.command_callback,10)
     
         def command_callback(self,msg):
             speed = 0.0
             if msg.data=="backup":
                 speed = -0.2
             self.get_logger().info(f'收到[{msg.data}]命令，发送速度{speed}')
     def main(args=None):
         rclpy.init()
         node = NodePublisher02("node_name")
         rclpy.spin(node)
         rclpy.shutdown()
     ```

### 2.3 服务



### 2.4 消息接口



### 2.5 参数

## 3 ROS2工具

### 3.1 launch启动编写

### 3.2 TF工具

### 3.3 RVIZ工具

### 3.4 URDF

### 3.5 gazebo工具
