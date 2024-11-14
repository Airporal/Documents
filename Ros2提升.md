# ROS2提升

> 介绍机器人学和ROS2特殊功能包用法





## 一：机器人学:robot:

### 1. 运动学

机器人学首先研究运动学。可以分为正运动学和逆运动学，正运动学指由控制施加力到运动状态变化的整个过程，逆运动学与之相反

在机器人控制中，机械臂的控制是最具有普适性的。一般常需要进行由状态量推算施加控制力的逆运动学分析。

机械臂的运动由**旋转矩阵**来表述。

#### 1.1 旋转矩阵

旋转矩阵首先是描述两组标准正交基的坐标系之间的旋转变换关系，可以概括为：
$$
[\eta_1,\eta_2,\eta_3]=[\xi_1,\xi_2,\xi_3]{}_B^A\!R
$$
其中：A系下的正交坐标基为：
$$
\xi_1、\xi_2、\xi_3
$$
B系下的正交坐标基为：
$$
\eta_1、\eta_2、\eta_3
$$
R为B系相对于A系的旋转矩阵。即正交坐标基在A系下的表示。

+ 坐标系的旋转矩阵是正交阵，自由度为3

+ 使用旋转矩阵表示将一个坐标系旋转变换：

将A系绕X轴旋转：
$$
A' =[\xi_1、\xi_2、\xi_3]\begin{bmatrix}
1 & 0 & 0 \\
0 & cos\alpha & -sin\alpha  \\ 0 & sin\alpha & cos\alpha 
\end{bmatrix}
$$
将A系绕Y轴旋转：
$$
A' =[\xi_1、\xi_2、\xi_3]\begin{bmatrix}
cos\beta & 0 & sin\alpha \\
0 & 1 & 0  \\ -sin\alpha & 0 & cos\beta 
\end{bmatrix}
$$
将A系绕Z轴旋转：
$$
A' =[\xi_1、\xi_2、\xi_3]\begin{bmatrix}
cos\gamma & -sin\gamma & 0 \\
sin\gamma & cos\gamma & 0  \\ 0 & 0 & 1 
\end{bmatrix}
$$
复杂的旋转可以由以上三种基本旋转组合。

+ fixed Angle的组合旋转

将一个坐标系绕着固定的坐标轴进行旋转。

**由于矩阵的乘法不满足交换率，因此，先绕x轴转20度再绕y轴转10度与先绕y轴转10度再绕x轴转20度得到的结果不一样。**

规定：fixed-Angle的旋转以X-Y-Z的顺序进行。由于旋转轴的固定，可以将整个旋转依次进行即可得到旋转矩阵。

旋转矩阵为：
$$
{}_B^AR_{XYZ}(\gamma,\beta,\alpha)
=\begin{bmatrix}c\alpha c\beta&c\alpha s\beta s\gamma-s\alpha c\gamma&c\alpha s\beta c\gamma+s\alpha s\gamma\\s\alpha c\beta&s\alpha s\beta s\gamma+c\alpha c\gamma&s\alpha s\beta c\gamma-c\alpha s\gamma\\-s\beta&c\beta s\gamma&c\beta c\gamma\end{bmatrix}=\begin{bmatrix}r_{11}&r_{12}&r_{13}\\r_{21}&r_{22}&r_{23}\\r_{31}&r_{32}&r_{33}\end{bmatrix}
$$
即：公式（4,5,6）依存左乘。

根据正交阵，可以由旋转矩阵的值解出以x、y、z为旋转方向的固定轴旋转的角度：


$$
if :\beta \neq 90^\circ:\\
\begin{aligned}
&\beta=Atan2(-r_{31},\sqrt{r_{11}{}^{2}+r_{21}{}^{2}}) \\
&\alpha=Atan2(r_{21}/c\beta,r_{11}/c\beta) \\
&\gamma=Atan2(r_3/c\beta,r_{33}/c\beta)
\end{aligned}
$$

$$
if :\beta = 90^\circ则\alpha=0^\circ,\gamma=
Atan2(r_{12},r_{22})
$$

$$
if :\beta = -90^\circ则\alpha=0^\circ,\gamma=
-Atan2(r_{12},r_{22})
$$

+ Euler Angle的组合旋转

当以动系的自身坐标轴为旋转轴时，成为Euler Angle旋转，一般知道旋转后的位置。

规定按照Z、Y、X的顺序旋转。Euler Angle角度解算时，如果正向思考旋转过程，由于角度不是相对于固定轴，因而难以计算在A系下的旋转矩阵，此时需要逆向思考整个过程，即先将B系绕X逆向旋转，再绕Y逆向旋转，最后绕Z逆向旋转，从而回到原位置，每次旋转相对于B系当前轴。
$$
{}_B^AR_{ZYX}(\alpha,\beta,\gamma)={}_{B_y}^AR_{Z}(\alpha,0,0){}_{B_x}^{B_y}R_{Y}(0,\beta,0){}_B^{B_x}R_{X}(0,0,\gamma)
$$
得到和fixed Angle一样的旋转矩阵表达，因此解算角度方法也一样。

==**若按照指定的顺序旋转，则fixed Angle下绕固定的轴旋转一定角度得到的结果和Euler Angle下绕变化的轴旋转相同的角度得到的结果一样**==



按照以上思想，在Euler Angle下，当其它旋转组合时，如绕z-绕y-绕z旋转，得到的旋转矩阵、旋转矩阵元素与角度对应关系都可以推出来。总共有12种旋转组合。

+ 使用旋转矩阵表示A系中点的旋转：

假设P点在A系中的坐标为(x,y,z)，求将它旋转后在A系中的坐标。此时虽然只研究一个坐标系，但是可以假设一个中间坐标系，中间坐标系施加指定的旋转后得到B系，而P随着一起旋转，固P在B系中的坐标为（x,y,z）则求其在A系中的坐标P’：
$$
[x',y',z'] = {}_B^A\!R[x,y,z]
$$

#### 1.2 运动表达

空间中的运动可以分解为旋转和平移两部分，分别使用运动坐标系相对于空间惯性系的旋转矩阵与运动坐标系原点位置表示。

![image-20240915104712115](https://cdn.jsdelivr.net/gh/Airporal/Pictures/img/202409151047178.png)

空间中点的运动为：
$$
{}^AP_{3\times1}={}^A_BR_{3\times3}*{}^BP_{3\times1}+{}^AP_{B_{org}3\times1}
$$
矩阵形式为：
$$
\begin{bmatrix}{}^AP\\1\end{bmatrix}=
\begin{bmatrix}{}^A_BR&&&{}^AP_{B org}\\0&0&0&1\end{bmatrix}\begin{bmatrix}{}^BP\\1\end{bmatrix}
$$
连续性的运动可拆解为连续性的运动矩阵相乘：
$$
{}^A_BT={}^A_CT{}^C_DT{}^D_BT
$$
与Euler Angle类似，连续的运动变化分析时，需要逆向分析整个过程。
$$
\begin{aligned}{{}^{A}P}&=
{}_{B}^{A}T_{C}^{B}T_{D}^{C}T^{D}P\\&
=\begin{bmatrix}{}_{B}^{A}R_{C}^{B}R_{D}^{C}R&&&{}^{A}P_{B org}+{}_{B}^{A}R^{B}P_{C org}+{}_{B}^{A}R_{C}^{B}R^{C}P_{D org}\\0&0&0&1\end{bmatrix}
={}_{D}^{A}T^{D}P\end{aligned}
$$
对于点自身的运动描述，先旋转然后位移，也可用以上方程描述。

运动的逆变化：旋转部分仍然为正交阵，逆就是转置；位移部分需要取相反的坐标原点关系：
$$
_A^BT^{}=_B^AT^{-1}=\begin{bmatrix}{}_B^AR^T&&&-{}_B^AR^T{}^AP_{B org}\\0&0&0&1\end{bmatrix}
$$

+ 结构的表达：

![image-20240915104721472](https://cdn.jsdelivr.net/gh/Airporal/Pictures/img/202409151047548.png)

杆件可以抽象为joint和link组成，对于每个杆件而言，描述的参数为：
$$
(a_{i-1},\alpha _{i-1},d_i,\theta_i)
$$
分别表示：

​	joint所在的轴和下一个杆件的轴的距离

​	joint所在的轴和下一个杆件的轴的夹角

​	joint轴上前后两个垂线之间的位置

​	joint轴上前后两个垂线之间的夹角

+ 杆件的Frame建立规则：

​	以转轴的方向为z的方向（移动副以移动方向），以与下一个杆件轴垂线方向为x的方向，y由右手定则确定，原点在与下一轴垂线的交点。角度方向以右手定则判断。

#### 1.3 顺运动学

![image-20240915104730368](https://cdn.jsdelivr.net/gh/Airporal/Pictures/img/202409151047429.png)

使用旋转矩阵进行相邻杆件之间的点坐标转换方法：

p在i杆的Frame已知，则在i-1杆的Frame上的坐标为：
$$
{}^iP={}^i_{i-1}T*{}^{i-1}P
$$
而由i-1系到i系的坐标系变化过程可以理解为：
$$
由i-1系绕x轴旋转\alpha_{i-1}->沿垂线平移a-1->绕z轴旋转\theta->沿z轴平移d_i
$$
即：
$$
{}^i_{i-1}T = {}^{i-1}_{R}T{}^R_{Q}T{}^Q_{P}T{}^P_{i}T
$$
代入1.2中的表达，得到两杆件之间的旋转矩阵为：
$$
\begin{bmatrix}c\theta_i&-s\theta_i&0&a_{i-1}\\s\theta_ic\alpha_{i-1}&c\theta_ic\alpha_{i-1}&-s\alpha_{i-1}&-s\alpha_{i-1}d_i\\s\theta_is\alpha_{i-1}&c\theta_is\alpha_{i-1}&c\alpha_{i-1}&c\alpha_{i-1}d_i\\0&0&0&1\end{bmatrix}
$$
同理，多个相邻的杆件之间的坐标变换，可以用使用各个旋转矩阵得到。

分析方法：

1. 画出所有的活动机构的Frame，个数为自由度-1
2. 画出开始和结尾部分的Frame
3. 以DH表格的形式，写出所有的Frame的参数，写的时候可以对Frame进行上文指定的变换以方便找到。
4. 根据参数得到旋转矩阵
5. 每个运动需要建立在两个接触杆件上的两个坐标系描述，总坐标轴数等与自由度+1

**通过控制各个关节的参数【Actuator Space]，进而控制末端位置【joint Space】的方法为顺运动学。**

#### 1.4 逆运动学

根据指定需要到达的坐标点，反算需要实现的关节运动。逆运动学解算实际上运用的更加广泛，如目标跟踪任务等。

+ 逆运动学解算往往可以得到多个解，可添加约束条件以获得需要的解
+ 解算思路：
  + 确定需要到达的点在world Frame下的坐标
  + 在该点下建立和最后一个关节Frame相平行的Frame
  + 计算该点与World Frame的转换矩阵、结构自身各个关节的转换矩阵
  + 由已知转换矩阵求解需要的旋转矩阵，进而确定运动姿态

能解系求取的一般只有前面几个运动参数，此时如果将最后的3个旋转轴设置为重合，则每次都是绕旋转后的z轴旋转，符合Euler Angle，则其旋转矩阵可以由Euler Angle解算出对应的运动参数。

注意点：

+ 除了结构自身关节的Frame外，可根据需要建立合适的Frame以方便求解分析
+ 根据转换矩阵为正交阵来求逆，已得到未知的转换矩阵
+ 根据转换矩阵中坐标与旋转的关系求解运动参数时，可设置合适的中间变量以方便求解。

#### 1.5 轨迹规划

根据起点与终点，确定空间中经过的点以及对应的转换矩阵，需要确保轨迹和速度的连续性，称为轨迹规划。

Joint-space下的轨迹规划：在各个关节上做规划，对姿态进行规划。

1. 转换矩阵具有十六个参数，而空间运动为三个自由度移动和三个自由度转动，直接使用转换矩阵会导致得到的规划结果不满足转换矩阵要求，所以需要将转换矩阵使用六个参数来表示。
2. I.K：根据手臂的目标末端点状态，得到各个关节的末状态。
3. Trajectories plan将所有的关节初状态、末状态规划为smooth trajectories。
4. F.K 根据各个关节的smooth trajectories推算手臂末端点的运动轨迹，验证轨迹可行性。

Certesian-space下的轨迹规划：直接对手臂末端点位置进行规划。

1. 用六个参数表示转换矩阵。

2. 将所有末端点状态规划为smooth trajectories

3. I.K将规划好的所有手臂末端轨迹经过的点转化为各个Joint的状态

4. 验证各个Joint是否能达到指定状态。

   Certesian-space下需要进行I.K次数太多，带来极大计算压力，但是轨迹更加直观。

Smooth trajectories 原则：

1. 相邻时间段的规划需要用不同参数的同类型函数拟合
2. 定义各个函数的边界条件，包括起点、终点各自的位置和速度
3. 通常以多项式来规划轨迹

**单段多项式单独规划举例：**
$$
\theta(\delta t)=a_0+a_1\delta t+a_2\delta t^2+a_3\delta t^3
\\t\in(t_i,t_{i+1}),\delta t\in[0,t_{i+1}-t_i],\Delta t =t_{i+1}-t_i
$$
通过四个边界条件解出多项式中四个未知数。得到矩阵运算为：
$$
\begin{bmatrix}\theta_i\\\theta_{i+1}\\\dot{\theta}_i\\\dot{\theta}_{i+1}\end{bmatrix}=\begin{bmatrix}1&0&0&0\\1&\Delta t&\Delta t^2&\Delta t^3\\0&1&0&0\\0&1&2\Delta t&3\Delta t^2\end{bmatrix}\begin{bmatrix}a_0\\a_1\\a_2\\a_3\end{bmatrix}
$$
当系数阵满秩序时，根据四个边界条件及最大的时间间隔可解上述方程。即行列式不为0：
$$
det(\begin{bmatrix}1&0&0&0\\1&\Delta t&\Delta t^2&\Delta t^3\\0&1&0&0\\0&1&2\Delta t&3\Delta t^2\end{bmatrix})=-\Delta t^4\neq 0 
$$
得到：
$$
\begin{bmatrix}a_0\\a_1\\a_2\\a_3\end{bmatrix}=\begin{bmatrix}1&0&0&0\\0&0&1&0\\-\frac{3}{\Delta t^2}&\frac{3}{\Delta t^2}&-\frac{2}{\Delta t}&-\frac{1}{\Delta t}\\\frac{2}{\Delta t^3}&-\frac{2}{\Delta t^2}&\frac{1}{\Delta t^2}&\frac{1}{\Delta t^2}\end{bmatrix}\begin{bmatrix}\theta_i\\\theta_{i+1}\\\dot{\theta}_i\\\dot{\theta}_{i+1}\end{bmatrix}
$$
初始条件设置准则：

a. 不建议直接单独对速度边界条件进行设定，因为不仅难以实现而且过于复杂。

b. 自动生成速各个时间点的速度边界条件：

+ 如果该点附近，速度变号，则设该点速度边界条件为0
+ 如果该点附近速度不变换，则设该点速度为前一点速度和后一点速度的平均值。

**多段单项式一起规划举例：**

对两端连续的两个多项式进行联立求解，共八个未知数，除了三个点的位置、速度共六个条件外，还有第一段右边界速度、加速度和第二段左边界速度、加速度相等。

最终求解得到矩阵方程：
$$
\begin{bmatrix}\theta_0\\\theta_1\\\theta_1\\\theta_f\\\theta_0\\\theta_f\\0\\0\end{bmatrix}=\begin{bmatrix}1&0&0&0&0&0&0&0\\1&\Delta t_1&\Delta t_1^2&\Delta t_1^3&0&0&0&0\\0&0&0&0&1&0&0&0\\0&0&0&0&1&\Delta t_2&\Delta t_2^2&\Delta t_2^3\\0&1&0&0&0&0&0&0\\0&0&0&0&0&1&2\Delta t_2&3\Delta t_2^2\\0&1&2\Delta t_1&3\Delta t_1^2&0&-1&0&0\\0&0&2&6\Delta t_1&0&0&-2&0\end{bmatrix}\begin{bmatrix}a_{10}\\a_{11}\\a_{12}\\a_{13}\\a_{20}\\a_{21}\\a_{22}\\a_{23}\end{bmatrix}
$$
有解条件为时间间隔不为0。

c. General Cubic Polynomials方法

+ 设总共有N+1个点，N段轨迹，中间点N-1个（可以是末端点的位置，也可以是关节的位置）

+ N段轨迹共有N个多项式、4N个参数需要求解，因此需要4N个方程
+ 每个中间点在两个轨迹上，因此得到2（N-1）个位置的方程，加上初始点和末端点各一个方程，共2N个
+ 中间点的速度、加速度连续，左右相同得到2（N-1）个方程
+ 剩下的两个方程：
  + 可以定义初始加速度=0、末端加速度=0得到两个方程
  + 可以定义初始速度、末端速度得到两个方程
  + 如果初始点和末端点重合，则定义速度、加速度相同，得到两个方程。

在使用以上a、b、c的方法得到多项式拟合的各个自由度运动关于时间的函数后，即可组成连续的轨迹，形成控制指令。

#### 1.6 速度平滑规划

轨迹规划时，要避免速度的突变，否则会造成加速度难以实现。

如果需要各个段匀速直线运动，则只需要确定速度和变化速度即可完成轨迹的规划。

通常将位置变化曲线的速度突变段，添加线性的二次函数，使得存在一个短暂的加速（或减速）过程。

设置方法：

+ 在速度突变点的领域内进行二次多项式平滑处理，通过前后速度值，与所设置的领域大小或者所设置的加速度大小，求得平滑的二次函数表达。
+ 通过多段函数的方式形成控制指令。
+ 由于真实joint受力复杂，难以实现精确的加速度控制，因此常常设置变速时间而不是加速度。
+ 二次平滑后，轨迹不再通过原来的点，可以在需要通过的点附近取其它点，以该点为二次平滑中点，使得最后平滑后经过需要通过的点。

规划的轨迹可能对于joint来说难以实现，因此需要结合joint来进行规划调整。

通过设置变化时间的方式进行速度规划：

+ 在轨迹经过的点的基础上，设置每段轨迹需要达到的速度，一般为位移除时间，首尾两端需要进行时间的平移。
+ 在速度的基础上，结合设置的变化时间，设置变化的加速度
+ 列出各个时间段对应的加速轨迹、匀速轨迹表达式
+ I.K得到关节的运动值，最终得到轨迹

### 2.动力学



