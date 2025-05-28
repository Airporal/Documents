# ROS基础知识

[TOC]

------



## 1.安装ROS

通过[鱼香ROS](https://fishros.org.cn/forum/topic/20/小鱼的一键安装系列?lang=zh-CN)的一键安装指令安装ros:rose::

```bash
wget http://fishros.com/install -O fishros && . fishros
```

rosdep安装：由于网络环境的问题无法安装成功。

可以直接安装鱼香肉丝的rosdepc.

也可以使用魔法安装：

首先查找rosdep位置：

```bash
whereis rosdep
```

🥹🫣打开该文件：​

```bash
sudo gedit /usr/bin/rosdep
```

使用魔法后，设置python代理在该文件头。

此时可以成功运行：

```bash
sudo rosdep init
```

终端设置代理后可运行：

```bash
rosdep update
```

此时安装成功。

## 2.使用ROS

> ROS用法与ROS2大致一致，ROS因为更加成熟，拥有更完善的教程和参考。

### 2.1创建发布者

> ***工作空间*** 是进行ROS项目的编程空间，需要我们创建src文件夹，并将源码功能包按照规范些在src文件夹下，使用ROS自带的catkin工具编译后，会在工作空间中生成install和devel和build文件夹，其中install文件夹存放发布给用户的最终文件，devel文件夹和install文件夹内容大致相同，测试编写时使用devel文件夹修改环境变量，build文件夹存放编译文件。

1. 新建catkin_ws/src文件夹作为工作空间

```bash
mkdir -p catkin_ws/src
```

-p表示递归创建

2. 初始化工作空间

```bash
cd catkin_ws
catkin_make
cd catkin_ws/src # 可不执行下面两行
catkin_init_workspace
```

首先在工作空间下新建devel文件和install文件

然后在src目录下建立一个CMakeLists.txt文件，这使得系统指导该catkin_src文件夹是一个ROS工作空间

3. 建立功能包

```bash
catkin_create_pkg pkg_name 依赖
```

在包名后面添加会用到的依赖，例如**roscpp、rospy、std_msgs**等

该命令会在src目录下新建一个pkg_name的文件夹，文件夹下会有新建的配置文件package.xml、CMakeLists.txt和include、src两个文件夹。

4. 编写ROS节点

**cpp**：在pkg_name文件夹下的src文件夹下新建自己的cpp（c++语法）并实现功能;

头文件至少需要包含ros和用到的数据类型：

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist/h>
```

建立一个main函数，程序将从此开始：

```cpp
int main(int argc, char **argv)
{
    //输出乱码时添加下句
    setlocale(LC_ALL,"");
    //ROS节点初始化
    ros::init(argc,argv,"节点名");
    //创建节点句柄,用来管理通信信息
    ros::NodeHandle n;
    //创建发布者，发布名为话题名的topic，消息类型为msgs，设置队列长度m用来处理缓存，来不及发的数据将放在m里，m不够时会丢弃最先入的数据
    ros::Publisher pub_name = n.advertise<msgs>("topicname",m);
    //设置循环频率为tHz,下面编写循环语句
    ros::Rate loop_rate(t);
    int count = 0;
    // ros::ok()--->>节点存在则返回true
    while(ros::ok())
    {
        //初始化消息实例，并根据消息实例的组成设置值
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = 0.5;
        vel_msg.angular.z = 0.2;
        //发布消息
        pub_name.publish(vel_msg);
        //相当于cout:
        ROS_INFO("I am a publisher,command are:
                 [%0.2f m/s ,%0.2f rad/s]",
                 vel_msg.linear.x,vel_msg.angular.z);
        //按照循环频率延时
        loop_rate.sleep();
    }
    
}
```

**python：**在pkg_name文件夹下新建文件夹scripts，将自己编写的python脚本放在该文件夹下。

```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
def velocity_publisher():
    # Ros节点初始化
    rospy.init_node()
    # 创建一个Publisher,指定消息类型，发布话题
    turtle_vel_pub - rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    # 设置循环的频率
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 初始化消息信息
        vel_msg = Twist()
        vel_msg.linear.x = 0.5
        vel_msg.angular.z = 0.2
        # 发布消息
        turtle_vel_pub.publish(vel_msg)
        rospy.loginfo()
        rate.sleep()
if __name__ == "__main__":
    try :
        velocity_publisher()
    except rospy.ROSInterruptException:
        pass
```

rospy.init_node函数说明：

```python
rospy.init_node(node_name,anonymous=False,log_level,disable_signals)
# anonymous表示是否自动生成节点名
# log_level表示日志级别，有DEBUG\INFO\WARN\ERROR\FATAL
# disable_signals表示是否禁用节点对信号的响应
```

5. 编译代码

**对于Cpp：**编译前需要配置CMakeLists.txt文件的编译规则

```cmake
# 将cpp编译为可执行文件
add_executable(${PROJECT_NAME}_node src/my_realsense_node.cpp)
# 链接到哪些cpp库
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
```

在工作空间根目录下编译：

```bash
# 使用catkin_make编译
catkin_make
# python3使用catkin build编译
catkin build
# 更新一下环境变量
source devel/setup.bash
```

**对于python：**先给脚本添加可执行权限

```bash
sudo chmod +x *.py
```

修改CMakeLists.txt文件如下：

```cmake
catkin_install_python(PROGRAMS
  scripts/name.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

6. 运行节点

```bash
# 先启动ros
roscore
# 启动节点
rosrun 功能包 节点名
```

### 2.2创建订阅者

1. 声明相关的包

2. 初始化ROS节点

3. 创建节点句柄

4. 创建Subscriber

5. 编写回调函数

6. 循环调用

7. 修改编译文件

   代码如下：

```cpp
#include <ros/ros.h>
#include <turtlesim/Pose.h>

// 一般回调函数以msg的常指针作为输入
void poseCallback(const turtlesin::Pose::ConstPtr& msg)
{
    //循环回调函数逻辑
    ROS_INFO("Turtle Pose : x:%0.6f,y:%0.6f",msg->x,msg->y)
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc,argv,"节点名");
    //创建节点句柄
    ros::NodeHandle n;
    //创建一个Subscriber,订阅topic，注册回调函数
    ros:Subscriber pose_sub = n.subscriber("topicname",10,poseCallback);
    //循环
    ros::spin();
    return 0;
    
}
```

编译文件修改如下：

```cmake
add_executable(${PROJECT_NAME}_node src/my_realsense_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

```

python编写：

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from turtlesim.msg import Pose

def poseCallback(msg):
	rospy.loginfo("Turtle pose : x:%0.6f,y:%0.6f",msg.x,msg.y)
def pose_subscriber():
	# 初始化节点
	rospy.init_node('name',anonymous=True)
	# 创建Subscriber 
	rospy.Subscriber("topic_name",Pose,poseCallback)
	# 循环
	rospy.spin()
if __name__ == "__main__":
	pose_subscriber()
```

+ **python回调函数传参**

  有时需要向回调函数中传递参数，除了msg可传递外，其它参数可通过以下方式传递：

  ```python
  # 封装到类，直接定义时使用self
  def callback(self,msg):
  	pass
  # 不封装，可通过全局变量使用参数
  a = 10
  def callback(msg):
  	print(a)
  # 不封装，可通过partial函数建立新的回调函数
  from functools import partial
  imageProcessCallback2 = partial(imageProcessCallback,a)
  def imageProcessCallback(a,msg):
  	print(a)
  ```

### 2.3 自定义消息

1. 在功能包下新建msg文件夹

   ```bash
   mkdir -p src/pkgname/msg/datatype.msg
   ```

2. 新建.msg文件

​	根据需要自定义消息类型

3. 编译依赖

​	在package.xml文件中添加build_depend和exec_depend

```xml
<!--   表示在编译时会用message_generation -->
<build_depend>message_generation</build_depend>
<!--   表示在运行时会用message_runtime -->
<exec_depend>message_runtime</exec_depend>
```

​	在cmakelists.txt文件中添加msg文件的编译命令：

在***find_package***里添加***message_generation***，如果编译错误检查此处

在***add_message_files***里添加msg文件名，表示编译此文件

在***generate_messages***里添加新建的msg基于哪些基本msg

在***catkin_package***里添加***message_runtime***，如果运行错误检查此处

​	添加完后可正常编译生成c++和python文件

4. 可调用文件路径

   msg生成的python调用文件路径：**devel/lib/python3/dist-packages/pkg_name/msg**

   msg生成的cpp调用文件路径：**devel/include**

5. 使用

   python中使用时，先from 功能包.msg import 消息名进行导入；之后实例化消息类，而消息的各个组成部分是该类的属性。

```python
from opencv_test.msg import coordinateData
coor = coordinateData()
```

​	**cpp中使用时，需要先配置vscode，然后导入节点包。**

​	vscode环境设置：

+ 避免vscode无法找到自建的消息，需要在C_cpp_properties.json下添加路径

+ 在includepath项中，添加path2devel/include/**

  ```json
  {
    "configurations": [
      {
        "name": "linux-gcc-x64",
        "includePath": [
          "${workspaceFolder}/**",
          "/opt/ros/humble/include/**",
          "/opt/ros/humble/**",
          "/usr/include/**"
        ],
        "defines": [],
        "compilerPath": "/usr/bin/gcc",
        "cStandard": "${default}",
        "cppStandard": "${default}",
        "intelliSenseMode": "linux-gcc-x64"
      }
    ],
    "version": 4
  }
  ```
  
  导入消息包：

```cpp
#include "opencv_test/coordinateData.h"
opencv_test::coordinateData coor;
```



### 2.4 launch文件编写

在功能包下新建launch文件夹，在里面新建launch文件。

launch文件本质是一个xml格式的文件，通过launch文件，可以设置节点参数、可以设置命名空间、可以复用代码和自动打开多个节点。

1. launch标签

在launch标签下编写launch文件。

```xml
<launch deprecated="节点弃用时报错语句">
	内容
</launch>
```

2. node

在node标签下打开新节点。

```xml
<launch deprecated="节点弃用时报错语句">
	<node pkg="功能包名" name="节点初始化的名字"
          type="可执行文化名或py文件名" output="log|screen"
          respawn="true/false" required="true/false"
          ns="namespace" args="xxx xxx xxx"
          machine="机器名" 
          />
</launch>
```

其中，**pkg、name、type**必须设置，name属性会覆盖代码文件中初始化节点的名。

**output**属性设置日志输出目标，log日志文件或screen屏幕

**respawn**设置此节点退出后是否自动重启

**required**设置此节点退出后杀死整个roslaunch

**ns**表示设置的命名空间，若设置，则此节点下的所有名称都会加上以该命名空间为前缀的字符串

**args**设置节点需要的参数

**machine**表示分布式部署时，让此节点运行在哪个设备上。

还可以设置子标签：

```xml
<launch deprecated="节点弃用时报错语句">
	<node pkg="功能包名" name="节点初始化的名字"
          type="可执行文化名或py文件名" output="log|screen"
          respawn="true/false" required="true/false"
          ns="namespace" args="xxx xxx xxx"
          machine="机器名" 
          >
        <env 设置环境变量/>
        <remap/ 重映射节点名>
        <rosparam/ 从yaml文件设置参数>
        <param/ 参数设置>
    </node>
</launch>
```

3. include 标签

方便launch文件的代码复用，可以使用include标签导入其它launch文件：

```xml
<launch deprecated="节点弃用时报错语句">
	<include file="$(find 功能包名)/launch文件在该功能包下的路径"，ns=""/>
</launch>
```

一般同时使用子标签arg传递参数：

```xml
<launch deprecated="节点弃用时报错语句">
	<include file="$(find 功能包名)/launch文件在该功能包下的路径"，ns="">
    	<arg name="" value="">
    </include>
</launch>
```

4. remap标签

当话题名称不匹配无法通信时，可以使用remap重映射话题名称。

```xml
<launch deprecated="节点弃用时报错语句">
	<remap from="" to=""/>
</launch>
```

5. param标签

在参数服务器上设置参数。

```xml
<launch deprecated="节点弃用时报错语句">
	<param name="" type="str/int/double/bool/yaml" value=""/>
</launch>
```

6. rosparam标签

从yaml文件导入参数或导出参数到yaml文件。

```xml
<launch deprecated="节点弃用时报错语句">
	<rosparam command="" file="$(find 功能包名)/yaml文件路径"/>
</launch>
```

command可以选择load/dump/delete（导入/导出/删除）,此标签会在所有其它标签之前执行。

7. group标签

可以对节点进行分组管理，方便对多个节点进行命名空间划分。

```xml
<launch deprecated="节点弃用时报错语句">
	<group ns="命名空间" clear_params="true/false"/>
</launch>
```

一般只通过arg调用设置命名空间。

8. arg标签

对launch文件进行动态调参，相当于文件的内部参数，不在参数服务器的管理中。

```xml
<launch deprecated="节点弃用时报错语句">
	<arg name="" value=""/>
	<arg name="" default=""/>
</launch>
```

使用value定义值时，外部调用launch文件不可更改该值。

+ 在调用launch文件时赋值：

```bash
roslaunch pkg-name name.launch argname:=value
```

+ 在launch文件内部调用：

```xml
<launch deprecated="节点弃用时报错语句">
	<arg name="arga" value="1"/>
	<param name="parama" type="str" value="$(arg arga)">
</launch>
```

launch文件编写完后，不需要编译可直接运行。

### 2.5 ROS服务

> 服务通信与打电话类型，服务双方可以实时接受对方的消息并发送应答。

服务通信中有至少三个对象：

+ ROSmaster —— 建立服务，注册服务双方信息
+ 服务端——创建服务
+ 客户端——订阅服务

和话题不同的是，必须先建立服务才能订阅。

新建功能包：

```
catkin_create_pkg pkg_name roscpp rospy std_msgs
```



#### 2.5.1 新建srv通信接口

在功能包目录下新建srv目录，并新建srv文件。格式为：

客户端发送的数据

\---

服务端发送的数据

```
int32 client1
int32 client2
---
int32 server1
```

#### 2.5.2 修改配置文件

修改cmakelist和package.xml文件：

cmakelist文件增加find_package和对srv文件的配置

```cmake
find_package(
	message_generation
)
add_service_files(
	name.srv
)
generate_messages(
	所设置的srv接口依赖的标准接口包
)
# 接触CAtkin_DEPENDS行注释，增加message_runtime
catkin_package(
	message_runtime
)
```

package.xml文件：

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

编译后，会在devel目录下生成对应的文件：

include文件夹下有cpp使用的头文件

lib/python3/dist-packages/功能包/srv目录下有py文件。

**按照话题接口的步骤，导入服务接口的vscode环境配置。**

#### 2.5.3 服务端创建

cpp创建步骤：

1. 导入头文件
2. 初始化ROS节点
3. 初始化节点句柄
4. 创建服务对象
5. 处理请求并产生相应
6. spin()

```cpp
// dairy头文件
#include "ros/ros.h"
#include "服务接口功能包/接口名.h"
// 服务处理函数以指针引用传参，一般固定为此
bool func(服务接口功能包::接口名::Request &request,
         服务接口功能包::接口名::Response &response){
    // 接收服务接口参数
    int num1 = request.num1;
    int num2 = request.num2;
    // 产生相应
    int sum = num1+num2;
    response.sum = sum;
    
}

int main(int argc,char *argv[]){
    // 初始化节点
    ros::init(argc,argv,"node_name");
    //初始化句柄
    ros::NodeHandle n;
    // 使用句柄建立服务端对象，并传入处理函数
    ros::ServiceServer server = n.advertiseService("服务名",func);
	// spin循环处理
    ros::spin();
    return 0;
}
```

7. 配置cmakelist

```cmake
add_executable(node_name,src/cppfile) 
add_dependencies(node_name ${PROJECT_NAME}_gencpp)
target_link_libraries(node_name)
```

python:

1. 导入接口包
2. 初始化节点
3. 创建服务端
4. 处理请求并产生相应
5. spin

```python
#!/usr/bin/python3
import rospy
from 功能包.srv import 接口名，接口名Request,接口名Response
def funcs(request):
    num1 = request.num1
    num2 = request.num2
    sum = num1+num2
    response = 接口名Response
    response.sum = sum
    return response
if __name__ == "__main__":
	rospy.init_node()
    server = rospy.Service("话题名"，消息类型，funcs)
    rospy.spin()
    
```

6. 添加可执行权限、在cmakelist中添加该python文件

#### 2.5.4客户端创建

cpp创建步骤：

1. 导入头文件
2. 初始化ROS节点
3. 创建句柄
4. 通过句柄创建客户端
5. 创建请求并处理响应
6. spin

```cpp
#include "ros/ros.h"
#include "服务接口功能包/接口名.h"

int main(int argc,char *argv[]){
    // 初始化节点
    ros::init(argc,argv,"node_name");
    ros::Nodehandle n;
    ros::ServiceClient client = n.serviceClient<服务接口功能包::接口名>("服务名");
    // 创建请求
    服务接口功能包::接口名 ask;
    ask.request.num1 = 100;
    ask.request.num2 = 200;
    // 等待服务发起
    client.waitForExistence();
    // 处理响应
    bool flag = client.call(ask);
    if(flag){
        ROS_INFO("响应成功")；
    }else{
        ROS_INFO("响应失败")；
    }
    return 0;
}
```

7. 修改cmakelist如上一节。

客户端需要再服务端创建后才能正常运行，否则会报错，可以使用以下函数在服务未发起时，让客户端等待：

```cpp
client.waitForExistence();
//或者以下函数：
ros::service::waitForService("服务名")
```

python实现步骤：

1. 导入包
2. 初始化
3. 创建客户端
4. 创建请求并处理响应
5. spin

```python
#!/usr/bin/python3
import rospy
from 功能包.srv import 接口名，接口名Request,接口名Response

if __name__ == "__main__":
	rospy.init_node()
    client = rospy.ServiceProxy("服务名"，服务接口)
	response = client.call(12,34)
    rospy.loginfo(f"{response.sum}")
    rospy.spin()
```

6. 修改cmakelist如上节。

命令行传入参数到客户端脚本：

```python
#!/usr/bin/python3
import rospy
from 功能包.srv import *
import sys

if __name__ == "__main__":
    # sys.argv储存参数,0是文件名
    if len(sys.argv!=3):
        sys.exit()
    num1 = int(sys.argv[1])
    num2 = iny(sys.argv[2])
	rospy.init_node()
    client = rospy.ServiceProxy("服务名"，服务接口)
    client.wait_for_service()
	response = client.call(num1,num2)
    rospy.loginfo(f"{response.sum}")
    rospy.spin()
```

客户端需要再服务端创建后才能正常运行，否则会报错，可以使用以下函数在服务未发起时，让客户端等待：

```python
client.wait_for_service()
# 或者
rospy.wait_for_service("服务名")
```

### 2.6参数服务器

> 参数服务器独立于工作节点，各个节点可以向参数服务器中写参数数据，也可以读取参数值。

参数服务器性能受限，最好用于静态参数设置。参数的操作有**增删改查**。

#### 2.6.1Cpp参数操作

2.6.1.1 新增与修改

1. 导入头文件

2. 初始化ros节点

3. 初始化句柄

4. 设置参数

   + 可使用nh.setParam(参数，值)
   + 可使用 ros::param::set(参数，值)

5. 修改参数

   重复设置相同参数会修改其值。


6. 删除参数
   + 可使用nh.deleteParam(参数)；
   + 可使用ros::param::del(参数)

7. 修改cmakelist

增加可执行文件及映射，设置节点

```cpp
#include "ros/ros.h"
int main(int argc, char *argv[]){
    // 初始化节点
    ros::init(argc,argv,"setparam");
    // 初始化句柄
    ros::NodeHandle nh;
    // 设置参照>>>使用句柄设置
    nh.setParm("para1_name",value1);
    // 设置参数>>>使用ros::parm设置
    ros::param::set("para2_name",value2);
    // 修改参照>>>使用句柄修改
    nh.setParm("para1_name",newvalue1);
    // 修改参数>>>使用ros::parm修改
    ros::param::set("para2_name",newvalue2);
    // 删除参数>>>使用句柄删除,删除成功返回True
    bool delete_win = nh::deleteParam("para1_name");
    // 删除参数>>>使用ros::param删除,删除成功返回True
    bool delete_win = ros::param::del("para1_name");
    return 0
       
}
```

2.6.1.2 参数 查询

+ 使用ros句柄查询参数

  + param(参数，默认值)：查询参数值，若不存在返回默认值。

  ```cpp
  ros::nodeHandle nh;
  // 如果存在该参数则返回其值，否则返回默认值
  double param1 = nh.param("para1_name",default_value)
  ```

  + getParam(参数，储存结果的变量)

  ```cpp
  doublr param1;
  // 如果存在则exit为True,并返回参数值给变量param1
  bool exit = nh.getParam("para1_name",param1)
  ```

  性能没有getParamCached好

  + getParamCached(参数，储存结果的变量)

  ```cpp
  doublr param1;
  // 如果存在则exit为True,并返回参数值给变量param1
  bool exit = nh.getParamCached("para1_name",param1)
  ```

  + hasParam(参数)

    ```cpp
    // 参数存在返回Ture
    bool exit = nh.hasParam("para1_name")
    ```

  + getParamNames(std::vector\<std::string>)

  ```cpp
  // 建立一个模板表
  std::vector<std::string> param_list;
  // 获取参数储存到表中
  nh.getParamNames(param_list);
  // 遍历参数表，并打印
  for(auto &&param : param_list){
  	ROS_INFO("遍历到的参数%s",param.c_str());
  }
  ```
  
  + searchParam(参数，储存搜索结果的变量)

  ```cpp
  std::string value;
  // 参数存在则在value中存入其值，不存在存入空
  nh.searchParam("para1_name",value);
  ```
  
+ 使用ros::Param查询参数

  使用的函数与句柄方式基本一样，参数也一样。

#### 2.6.2 python参数操作

1. 导入包

2. 初始化节点

3. 新增参数

   + rospy.set_param(参数，值)

4. 修改参数

   + rospy.set_param(参数，新值)

5. 删除参数

   + rospy.delete_param(参数)

   参数不存在时，因无法删除而报错

6. 查询参数
   + rospy.get_param(参数，默认值)
   + rospy.get_param_cached(参数，默认值)
   + rospy.get_param_names()
   + rospy.has_param(参数)
   + rospy.search_param(参数)

```python
#！ /usr/bin/python3
import rospy
if __name__ == "__main__":
	rospy.init_node("node_name")
    # 设置参数
    rospy.set_param("param_name",value)
    # 修改参数
    rospy.set_param("param_name",value)
    # 查询参数
    value = rospy.get_param("param_name")# 直接查询参数
    value_cached = rospy.get_param_cached("param_name")# 从缓存中查询，效率更高
    names = rospy.get_param_names() # 返回参数list
    exit = rospy.has_param("param_name") # 查询参数是否存在
    value = rospy.search_param("param_name") # 搜索参数
    # 删除参数
    try:
        rospy.delete_param("param_name")
    except Exception as e:
        rospy.loginfo("被删除的参数不存在！")
    
```

7. 配置cmake_list与可执行权限

### 2.7 ros命令行工具



| 命令                          | 功能                               |
| ----------------------------- | ---------------------------------- |
| rosnode ping node_name        | 查看节点连接性                     |
| rosnode list                  | 列出所有节点                       |
| rosnode info node_name        | 查看节点信息                       |
| rosnode machine 设备名        | 查看某台设备上运行的节点           |
| rosnode kill node_name        | 杀死节点                           |
| rosnode cleanup               | 清除不可ping节点                   |
|                               |                                    |
| rostopic list                 | 列出活动话题                       |
| rostopic echo 话题名          | 订阅该话题输出                     |
| rostopic pub 话题名 参数 -r n | 每秒n次发布该话题数据              |
| rostopic info 话题名          | 查看话题信息                       |
| rostopic hz 话题名            | 打印话题发布频率                   |
| rostopic type 话题名          | 打印话题类型                       |
| rostopic find 接口名          | 根据数据接口查找话题               |
| rostopic bw 话题名            | 显示话题带宽                       |
| rostopic echo 话题名>文件名   | 可以将话题订阅并保存到本地         |
|                               |                                    |
| rosservice list               | 列出所有服务                       |
| rosservice call 服务名 数据   | 发送服务                           |
| rosservice info 服务名        | 查看服务信息                       |
| rosservice type 服务名        | 打印服务参数接口类型               |
| rosservice find 接口名        | 根据接口查找服务                   |
|                               |                                    |
| rosmsg list                   | 列出所有消息类型                   |
| rosmsg info 消息名            | 查看消息具体信息                   |
| rosmsg md5 消息名             | 显示md5加密的消息                  |
| rosmsg package 功能包名       | 显示某个功能包下所有消息           |
| rosmsg packages               | 列出包含消息的功能包               |
|                               |                                    |
| rossrv list                   | 显示所有服务消息                   |
| rossrv info 服务消息名        | 显示服务消息具体信息               |
|                               |                                    |
| rosparam set 参数名 值        | 设置\修改参数                      |
| rosparam get 参数             | 获取参数                           |
| rosparam load filename.yaml   | 将磁盘文件中参数加载到缓存         |
| rosparam list                 | 列出参数服务器里所有参数           |
| rosparam delete 参数名        | 删除参数                           |
| rosparam dump filename.yaml   | 将参数写入磁盘文件(序列化yaml格式) |
|                               |                                    |
|                               |                                    |

## 3.ROS工具

### 3.1 TF坐标管理工具

> TF2是ROS中用来管理和传输坐标信息的功能包，可以实现不同坐标系之间的点或者向量的转换

TF中的坐标msg消息常用以下两种：

geometry_msgs/TransformStamped: 传输坐标系相关信息；

```bash
std_msgs/Header header                     #头信息
  uint32 seq                                #|-- 序列号
  time stamp                                #|-- 时间戳
  string frame_id                            #|-- 坐标 ID
string child_frame_id                    #子坐标系的 id
geometry_msgs/Transform transform        #坐标信息
  geometry_msgs/Vector3 translation        #偏移量
    float64 x                                #|-- X 方向的偏移量
    float64 y                                #|-- Y 方向的偏移量
    float64 z                                #|-- Z 方向上的偏移量
  geometry_msgs/Quaternion rotation        #四元数
    float64 x                                
    float64 y                                
    float64 z                                
    float64 w

```

geometry_msgs/PointStamped:传输坐标系内点的信息。

```bash
std_msgs/Header header                    #头
  uint32 seq                                #|-- 序号
  time stamp                                #|-- 时间戳
  string frame_id                            #|-- 所属坐标系的 id
geometry_msgs/Point point                #点坐标
  float64 x                                    #|-- x y z 坐标
  float64 y
  float64 z

```

**坐标旋转关系使用四元数表示**。

#### 3.1.1 cpp静态坐标变换

两个坐标系之间的相对关系是固定时，坐标在它们之间变换为**静态坐标变换**。

若laser坐标系相对于base_link坐标系平移（0.2,0.0,0.5）,则使用TF静态坐标变换例程如下：

新建tf功能包：

```bash
catkin_create_pkg roscpp rospy std_msgs tf2tf2_ros tf2_geometry_msgs geometry_msgs
```

添加cmakelist：和普通包的cmakelist修改方式一样

设置静态坐标转换发布者：

**发布者坐标系之间的关系数据**

```cpp
#include "ros/ros.h"
// 静态坐标发布包
#include "tf2_ros/static_transform_broadcaster.h"
// 坐标转换数据接口
#include "geometry_msgs/TransformStamped.h"
// 四元数处理包
#include "tf2/LinearMath/Quaternion.h"

int main(int argc,char *argv[])
{
    // 设置编码格式
    setlocale(LC_ALL,"");
    // 初始化节点
    ros::init(argc,argv,"static_brocast");
    // 初始化静态坐标转换广播器
    tf2_ros::StaticTransformBroadcaster broadcaster;
    // 初始化坐标数据接口
    geometry_msgs::TransformStamped ts;
    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link";

    ts.child_frame_id = "laser";
    ts.transform.translation.x = 0.2;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.5;
	// 实例化四元数转换器
    tf2::Quaternion qtn;
    // 设置给定的欧拉角值，自动得到四元数表示
    qtn.setRPY(0,0,0);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
	// 发布静态坐标转换消息
    broadcaster.sendTransform(ts);
    // 循环发布
    ros::spin();
    return 0;

}
```

静态坐标订阅者

1. 导入包
2. 初始化节点、nodehandle
3. 初始化buffer：用来储存订阅到的坐标信息
4. 初始化listener：监听tf话题，并存入buffer
5. 初始化pointstamped信息，设置点的坐标系、坐标
6. 编写坐标转换逻辑
   1. 初始化转换后的坐标pointstamped
   2. 使用buffer.transform转换坐标值
   3. ros循环

```cpp
#include "ros/ros.h"
// 坐标信息接口
#include "geometry_msgs/PointStamped.h"
// listen
#include "tf2_ros/transform_listener.h"
// 坐标转换时用，必须添加
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    // 设置编码格式
    setlocale(LC_ALL, "");
    // 初始化节点
    ros::init(argc, argv, "static_subscriber");
    // 必须有nh
    ros::NodeHandle nh;
    // 储存接收到的tf坐标系话题TransformStamped
    tf2_ros::Buffer buffer;
    // 监听tf话题,并将值存入buffer
    tf2_ros::TransformListener listener(buffer);
    // 设置某个坐标系下的某点坐标数据
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser";
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    // 休眠等待一下，否则坐标关系还未收到就转换会报错
    ros::Duration(2).sleep();
    // 设置循环频率
    ros::Rate rate(10);
    while (ros::ok())
    {	// 转换后的坐标
        geometry_msgs::PointStamped sp;
        // 转换函数，输入坐标和转到的坐标系
        sp = buffer.transform(ps, "base_link");
        ROS_INFO("转换后的坐标：(%.2f, %.2f, %.2f),\t 参考坐标系为：%s",
                sp.point.x, sp.point.y, sp.point.z, sp.header.frame_id.c_str());
        // 休眠，循环执行
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}
```

使用try捕获异常并等待：

```cpp
#include "ros/ros.h"
// 坐标信息接口
#include "geometry_msgs/PointStamped.h"
// listen
#include "tf2_ros/transform_listener.h"
// 坐标转换时用，必须添加
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    // 设置编码格式
    setlocale(LC_ALL, "");
    // 初始化节点
    ros::init(argc, argv, "static_subscriber");
    // 必须有nh
    ros::NodeHandle nh;
    // 储存接收到的tf坐标系话题TransformStamped
    tf2_ros::Buffer buffer;
    // 监听tf话题,并将值存入buffer
    tf2_ros::TransformListener listener(buffer);
    // 设置某个坐标系下的某点坐标数据
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser";
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    // 休眠等待一下，否则坐标关系还未收到就转换会报错
    // ros::Duration(2).sleep();
    // 设置循环频率
    ros::Rate rate(10);
    while (ros::ok())
    {	// 转换后的坐标
        geometry_msgs::PointStamped sp;
        // 转换函数，输入坐标和转到的坐标系
        try
        {
            sp = buffer.transform(ps, "base_link");
        ROS_INFO("转换后的坐标：(%.2f, %.2f, %.2f),\t 参考坐标系为：%s",
            sp.point.x, sp.point.y, sp.point.z, sp.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            ROS_INFO("异常：%s",e.what());
        }
        
        
        // 休眠，循环执行
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}
```

#### 3.1.2 py静态坐标转换

发布者：发布坐标系关系

1. 导入依赖包
2. 创建ros节点
3. 创建tf发布对象
4. 设置发布消息，使用tf包进行四元数转换
5. 发布消息
6. 循环发布

```python
#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

import tf

if __name__ == "__main__":
    rospy.init_node("tf_pub")
    pub = tf2_ros.StaticTransformBroadcaster()
    ts = TransformStamped()
    ts.header.frame_id = "base_link"
    ts.child_frame_id = "laser"
    ts.header.stamp = rospy.Time.now()
    ts.transform.translation.x = 2.0
    ts.transform.translation.y = 0.0
    ts.transform.translation.z = 0.5

    ts.transform.rotation.x = tf.transformations.quaternion_from_euler(0, 0, 0)[0]
    ts.transform.rotation.y = tf.transformations.quaternion_from_euler(0, 0, 0)[1]
    ts.transform.rotation.z = tf.transformations.quaternion_from_euler(0, 0, 0)[2]
    ts.transform.rotation.w = tf.transformations.quaternion_from_euler(0, 0, 0)[3]
    pub.sendTransform(ts)
    rospy.spin()

```

订阅者：订阅坐标系关系，并转换坐标

1. 导入包

2. 初始化节点

3. 初始化buffer

4. 初始化listener并将接受到的信息存入buffer

5. 初始化坐标数据并赋值

6. 编写坐标转换逻辑

   使用buffer.transform转换坐标

```python
#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs.tf2_geometry_msgs


if __name__ == "__main__":
    rospy.init_node("tf_sub")
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    ps = PointStamped()
    ps.header.frame_id = "laser"
    ps.header.stamp = rospy.Time(0)
    ps.point.x = 2.0    
    ps.point.y = 3.0
    ps.point.z = 5.0    
    # rospy.sleep(1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            sp = buffer.transform(ps, "base_link")
            rospy.loginfo("Point in %s frame: (%.2f, %.2f, %.2f)",sp.header.frame_id,sp.point.x, sp.point.y, sp.point.z)
            

        except Exception as e:
            rospy.logerr(e)
        rate.sleep()
    

```

#### 3.1.3 命令行发布静态坐标转换

​	直接通过命令行发布静态坐标关系

```bash
# 发布两个相对静止坐标系关系
rosrun tf2_ros static_transform_publisher x y z r q w /base_link /camera

```

**静态坐标发布一般采用此方式**。

#### 3.1.4 cpp动态坐标变换

相比于固定不动的坐标关系，动态变换的坐标关系更为常见。即机器人运动时，自身坐标系相对于世界坐标系是变换的，因此需要动态处理坐标关系。

以乌龟节点为例，建立乌龟坐标系与世界坐标系之间的动态坐标变换

+ 动态坐标发布者

使用tf需要的功能包和turtlesimg功能包：

```
roscpp rospy std_msgs tf2 tf2_geometry_msgs tf2_ros turtlesim
```

修改cmakelist。

建立发布者的过程和静态坐标系的发布者大致一样，只是用到的函数接口名有所不同：

1. 包含头文件和主函数、设置编码格式
2. 初始化节点
3. 初始化句柄
4. 建立订阅者，订阅乌龟坐标消息
5. 写回调函数逻辑
   1. 传入消息的静态指针引用
   2. 初始化tf广播器和tf消息
   3. 填充tf消息，使用四元数转换角度值
   4. 发布tf消息
6. spin

```cpp
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
/*
    订阅乌龟节点的位置信息，转换为相对于窗体的坐标并发布
    运行乌龟节点，rosrun turtlesim turtlesim_node
    订阅话题：/turtle1/pose
    话题消息接口为:/turltesim/Pose

*/
void doPose(const turtlesim::Pose::ConstPtr& msg){
    // 创建一个tf广播器
    static tf2_ros::TransformBroadcaster pub;
    // 创建一个tf消息
    geometry_msgs::TransformStamped ts;
    // 填充tf消息
    ts.header.stamp = ros::Time::now();
    // 乌龟要转到的坐标系
    ts.header.frame_id = "world";
    // 乌龟的坐标系
    ts.child_frame_id = "turtle1";
    ts.transform.translation.x = msg->x;
    ts.transform.translation.y = msg->y;
    ts.transform.translation.z = 0;
    // 四元数表示法
    tf2::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    ts.transform.rotation.x = q.getX();
    ts.transform.rotation.y = q.getY();
    ts.transform.rotation.z = q.getZ();
    ts.transform.rotation.w = q.getW();
    // 发布tf消息
    pub.sendTransform(ts);
    }

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tf_dynamic_pub");
    ros::NodeHandle nh;
    // 订阅乌龟节点的位置信息，在回调函数中处理并发布
    
    ros::Subscriber sub = nh.subscribe("/turtle1/pose", 100, doPose);
    ros::spin();
    return 0;
}

```

+ 动态坐标订阅者

基本和静态坐标订阅者一样，只不过在发布消息时，时间戳必须设置为**ros::Time(0,0)，**不能设置为now，因为会出现转换前后now时间不匹配而报错。

```cpp
#include "ros/ros.h"
// 坐标信息接口
#include "geometry_msgs/PointStamped.h"
// listen
#include "tf2_ros/transform_listener.h"
// 坐标转换时用，必须添加
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    // 设置编码格式
    setlocale(LC_ALL, "");
    // 初始化节点
    ros::init(argc, argv, "dynamic_subscriber");
    // 必须有nh
    ros::NodeHandle nh;
    // 储存接收到的tf坐标系话题TransformStamped
    tf2_ros::Buffer buffer;
    // 监听tf话题,并将值存入buffer
    tf2_ros::TransformListener listener(buffer);
    // 设置某个坐标系下的某点坐标数据
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "turtle";
    // ps.header.stamp = ros::Time::now();
    ps.header.stamp = ros::Time(0,0);
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;
    // 休眠等待一下，否则坐标关系还未收到就转换会报错
    // ros::Duration(2).sleep();
    // 设置循环频率
    ros::Rate rate(10);
    while (ros::ok())
    {	// 转换后的坐标
        geometry_msgs::PointStamped sp;
        // 转换函数，输入坐标和转到的坐标系
        try
        {
            sp = buffer.transform(ps, "world");
        ROS_INFO("转换后的坐标：(%.2f, %.2f, %.2f),\t 参考坐标系为：%s",
            sp.point.x, sp.point.y, sp.point.z, sp.header.frame_id.c_str());
        }
        catch(const std::exception& e)
        {
            ROS_INFO("异常：%s",e.what());
        }
        
        
        // 休眠，循环执行
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}
```

## 3.2 rivz



## 3.3 gazebo

> gazebo是一个独立于ros的仿真软件，提供与ros交互的功能包，使用时，通过launch文件打开一些gazebo的节点，然后设置自己的仿真模型即可。

+ spawm_model

向gazebo中载入urdf模型。

```xml
  <node
    name="spawn_model"
    pkg="gazebo_ros"
    type="spawn_model"
    args="-file $(find hroerone)/urdf/hroerone.urdf -urdf -model hroerone"
    output="screen" />
```

使用以下方法查看节点使用的参数：

```bash
rosrun gazebo_ros spawn_model -h
```



+ 颜色设置

gazebo不直接使用rivz中显示的颜色标签，而需要使用gazebo的标签在urdf文件中重新设置颜色。

```xml
<gazebo reference="link节点名称">
	<material>Gazebo/Blue</material>
</gazebo>
```





### 3.4 Ros_Control

> ros_control是一组包含了控制器接口、控制器管理器、传输和硬件接口的软件包。用于控制机器人在仿真环境或真实环境中的运动





















































