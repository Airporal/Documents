# 记录QT语法及使用技巧





------

# 0：qt学前准备

1. linux下命令行使用g++编译cpp文件：

```bash
// 编译生成可执行文件name.o
g++ name.cpp -o name.o
// 运行可执行文件name.o
./name.o
```

2. 调用可执行文件时动态传参

```cpp
int main(int argc,char *argv[]){
	//argc 记录参数个数，如，rosrun xxx xxx 12 34，则argc==3
    //argv 储存参数值, argv[1]="12",argv[2]="34"
    
}
```

3. 循环遍历vector

+ vector是std库中的一个模板，使用for遍历：

```cpp
std::vector<std::string> vector;
for (auto &&x : vector){
	printf("X value is %s",x.c_str());
}
```

4. cpp原则

   一般在h文件中声明用到的对象，在cpp文件中进行定义，在主程序中调用。

5. qt安装

   参考常用软件安装

6. 使用vs2022创建QT项目

+ 打开vs2022,点击创建新项目
  ![image-20240926164429446](https://gitee.com/airporal/image_hub/raw/master/img/202409261644524.png)

+ 选择创建QT widgets Application 项目

  ![image-20240926164552997](https://gitee.com/airporal/image_hub/raw/master/img/202409261645036.png)

  QT有三大基类，分别为QWidget、QMainWindow、QDialog。

  <font color='red'>**QMainWindow**</font>是主窗口界面，不包含任何东西。

  <font color='red'>**QWidget**</font>是带菜单栏的界面，继承自主窗口。

  <font color='red'>**QDialog**</font>是带对话框的界面，继承自主窗口。

+ 设置项目名称和位置，名称将作为建立的类名，需要英文。

  ![image-20240926164648216](https://gitee.com/airporal/image_hub/raw/master/img/202409261646251.png)

+ 点击创建，出现弹窗，点击next后出现下面窗口，选择debug模式的√

  ![image-20240926164827659](https://gitee.com/airporal/image_hub/raw/master/img/202409261648692.png)

+ 点击finish完成创建

  ![image-20240926164858429](https://gitee.com/airporal/image_hub/raw/master/img/202409261648461.png)

---

7. 命名规范

   首字母大写，单词和单词之间首字母大写

   函数名、变量名首字母小写，单词和单词之间首字母大写

8. 在QT assistant中可以查看各个组件的用法。

   ![image-20240927143814949](https://gitee.com/airporal/image_hub/raw/master/img/202409271438055.png)

9. 新建Qt组件

	一般使用一个新的cpp和头文件来新建一个新的组件，使用Visual Studio 新建类：
	
	右键点击源文件栏，选择添加-->选择类：

![image-20240928153556227](https://gitee.com/airporal/image_hub/raw/master/img/202409281536323.png)

设置类名和基类及继承类型即可自动生成cpp和h文件。

![image-20240928153737685](https://gitee.com/airporal/image_hub/raw/master/img/202409281537725.png)

可在**头文件**中添加以下模板：

```c++
#pragma once // 防止头文件多次被包含的简单方法
// 用到的父类的头文件
#include <qwidget.h> 
#include <QPushButton>

class MyPushButton :
    public QPushButton
{
    Q_OBJECT // QT宏定义，添加以使用信号等
public:
    // explicit 禁止隐式转换的关键字，用来提高代码安全性
	explicit MyPushButton(QWidget* parent = nullptr); 

signals://声明信号用

public slots: //处理信号的槽用
};S


```

10. 项目编写原则

    组件各自包含自己的h和cpp文件，声明在h文件，实现在cpp文件。

    包含其它组件时，需要再h文件中声明。

# 一. C++

## 1.输出乱码问题

在main函数下增加以下内容即可：


```cpp
setlocale(LC_ALL,"");
```

## 2.循环调用问题

可使用定时器回调函数，可直接设置循环。

使用回调函数时：

```cpp
//封装到类的需要绑定回调函数与this指针
// 由于需要再类的内部调用该函数，所以传入this，每次接收两个参数，则设置两个占位符
std::bind(&fun,this,_1,_2)
// 直接调用的无需绑定，可正常定义即可
// 传入消息指针为参数
```

## 3.c++三目运算符

```c++
int a =3;
int b =4;
int c;
// 将a和b中较大的赋值给c
c = a>b?a:b;
// 将a和b中较小的赋值为0
a<b?a:b = 0;

```

## 4.指针

> windowsX64下，任何指针都只占4个字节，使用指针作为参数传递可以节约内存

```c++
c++中，空指针NULL用来初始化指针变量
空指针不能操作（取值等）
野指针（随便取的十六进制地址）不能操作
指针常量
    int * const p =&a :
	// 指针指向的变量的值可以更改
	*p = 20
    // 不可以修改指针的指向
    p = &b
常量指针
    const int * p = &a :
    // 可以修改指针的指向
    p = &b; 
    // 不可以修改指针指向的变量的值
    *p = 20; //错
常量指针常量
    const int * const p = &a
    都不可修改
数组指针
    数组的名就是数组的指针也是数组第一个元素的地址。
    以数组的指针作为函数参数传递，需要同时传递数组的长度
```

## 5.结构体

```c++
// 结构体定义
struct Student {
	int age;
	string name;
	string sex;
	int score;
}; // 可在定义结构体内容的同时声明一个结构体
// 结构体使用点运算符访问成员值、赋值。声明结构体实例时，可以加struct 也可以直接写Student。
	Student moon;
	moon.age = 23;
	moon.name = "moon";
	moon.score = 100;
// 也可以这样初始化（值和定义要对应）：
	Student le = {22,"le","girl",100};	
// 结构体数组
	struct Student students[] = {
		{23,"chen","male",100},
        {21,"Airporal","male",120}
	};
// 结构体指针，需要通过->符号来通过结构体指针访问成员值
	Student lee = {22,"le","girl",100};
	Student* le = &lee; // 结构体名不是其地址，必须用取地址符号&
	cout << "名字" << le->name << "年纪" << le->age << "分数" << le->score << endl;

// 函数以结构体为参数：
void printInfor(Student s);  // 值传递，使用点运算符访问成员，不修改值
void printInfor(Student* s);  // 地址传递，使用剪头运算符访问成员，可修改值
void printInfor(Student s[],int len); //结构体数组也可以用数组名
// const修饰结构体
// 以地址传递可以节约内存，但是可能会错误修改值，使用const修饰的常量结构体指针作为参数列表可以避免修改成员。
void printInfor(const Student* s); // 若修改会提示，不可修改的左值。


// c++结构体和class都可以声明一个类，但是struct默认为公共，class默认为私由
```

## 6.时间与生成随机数

```c++
#include <ctime>
srand((unsigned int)time(NULL));  // 以系统时间生成随机数种子
int random = rand(); //产生一个随机整数
```

## 7.内存四区

C++ 程序运行过程中内存划分为四个区域：

**代码区：**使用二进制储存代码，可以共享调用；

**全局区：**双引号字符串、全局变量、常量（static修饰的静态变量和const修饰的全局变量）

**栈区：**编译器自动释放内存，包括函数的参数值、局部变量等。不要用局部变量的地址作为函数返回值，第一次编译器保留下来可以返回，之后就会自动释放。

**堆区：**人为决定是否释放，如果不释放，在代码运行结束后会自动释放。

```cpp
// 使用new关键字 可以将数据开辟到堆区。本质上就是在堆区建立变量,将地址返回给栈区的变量，使得编译器不能自己释放，此时可以作为函数返回值
int *p = new int(10);
// 释放推上的内存
delete p;

// new 一个数组
int *p = new int[10];
// 正常使用数组
p[1] = 1;
// 释放堆上数组
delete[] p;

```

## 8.c++引用（符号&）

```c++
// 应用即为变量创建一个别名（指针）,应用一经过创建,需要同时初始化，且之后不可更改其它别名，变量的不同别名指向同一个内存地址。
int a =10;
// 系统内部自动声明：int *const b = a; b是一个指针常量,因而不可任意改变指向。
int &b = a;
// 在使用时，发现b是引用,自动转换为 *b = 20
b = 20;
int & c = 1;// 报错，不能指向不合法空间

const int &c = 2; // 不报错，因为编译器会自动修改为int temp = 2, const int 
c= 3;//报错，const 不能修改

// 引用作为函数参数
// 引用作为函数参数时，相当于将实参换别名进行操作，可以实现指针一样的效果
int fun(int &a,int &b){
}

// 引用作为函数的返回值，不能返回局部变量的引用，在函数结束后会释放内存，使得返回乱码，可以将局部变量声明为静态变量放置全局区从而解决

int& fun(){
    static int a = 19;
    return a;
}

// 函数以常量引用作为形参可以防止内部错误修改
int& fun(const int & a){
    static int a = 19;
    return a;
}

// 引用作为函数形参时，不带const的参数不能直接用数调用,加const才能调用
fun(10); // 常量引用形参时才 合法



int main(){
    int &b = fun();
    cout << b<<endl; // 19
    // 只有当返回引用时，函数才能作为等号的左值
    
    fun() = 1;
    cout<< b <<endl //1
    
}

```

## 9.函数高级

+ 函数形参某个参数有了默认参数，则后面位置的参数必须都有默认参数

```c++
// 错误
int fun(int a, int b=10, int c ){

}
//正确
int fun(int a, int b=10, int c=20 ){

}
```

+ 声明和实现只能有一个有默认参数。
+ 占位参数

```c++
// 函数形参中使用一个数据类型而不加形参名，则参数中包含一个占位参数
void fun(int a,float){
    
}

```

+ 函数重载

函数参数的个数、顺序、类型的不同可以声明不同的函数重载。

```c++
// 引用作为重载，引用作为
```

## 10.类的构造函数和析构函数

> 编译器默认提供默认构造、析构构造、拷贝构造函数。如果写了有参则不提供无参，如果写了拷贝，则不提供其它的构造函数。

```c++
/* 
构造函数和析构函数都写在public下
构造函数有参数，可以重载
析构函数没有参数，不可以重载
*/
class Student
{
private:
	int age;
	string name;
	int score;
public:
	void setAge(int age);
	void setName(string name);
	void setScore(int score);
	Student() {
		cout << "Student()" << endl;
	}
    
	Student(string name) {
		cout << "Student"<<name << endl;
	}
    // 初始化列表构造
    Student(int age，string name，int score):age(age),name(name),score(score)
    {
    cout << "Student"<<name << endl;
    }
// 拷贝构造函数：将一个已经实例化的类作为参数，以常量引用的形式传入构造函数中，将该类的属性拷贝到新的类中
    Student(const Student& p ){
        age = p.age
    }
	~Student() {
		cout << "~Student()" << endl;
	}
};
```

类实例的构造方法

```c++
// 括号法
Student s1; // 使用默认构造函数构造
Student s2(); // 错误，默认构造函数不加括号
Student s3(s1); // 拷贝构造
Student s4("Airporal"); // 传值构造

// 等号法+匿名函数
// Student(s1)会创建一个匿名对象，不用等号设置名字则会释放。
Student s5 = Student(s1);
Student s6 = Student("Airporal");

// 赋值
Student s7 = s1;
Student s8 = "Airporal";
```

拷贝构造函数使用

```c++
// 1. 通过一个实例化的类构建一个新的类
Student p2(p1);

// 2. 值传递，类作为函数形参时，会先调用拷贝构造函数
void classInit(Student p){

}

// 3. 返回一个类时会调用拷贝构造函数
Student classInit(){

}

// 类使用堆区内存时，需要在析构函数中释放内存，如果使用默认的拷贝构造函数，只会进行浅拷贝，使得两个类的实例使用同一个堆的数，释放时重复释放导致崩溃。
	Student(const Student &p) {
        // 深拷贝，重新申请一个堆区地址给新构造的类实例
		this->score = new int(*p.score);
		cout << "Student(const Student &p)"<<this->age << endl;
	}
	~Student() {
		if (score != NULL) {
			delete score;
			score = NULL;
		}
		cout << "~Student()" << endl;
	}
```

## 11.类的静态成员

静态成员变量：

+ 全局使用，所有成员共享同一个数据
+ 类内声明，类外定义
+ 在编译阶段分配内存

```c++
static int day; // 声明
int Student::day = 0; // 定义
// 访问
cout << s1.day << endl;
s2.day = 2;
cout << s1.day << endl;
cout << Student::day << endl;
```

静态成员函数：

+ 所有对象共享同一个数据
+ 只能使用静态成员变量作为参数
+ 类内声明，类内定义

```c++
static void getDay() {
    cout << day << endl;	}
    s1.getDay();
s2.day = 2;
cout << s1.day << endl;
Student::getDay();
system("pause");
return 0;
```

类的成员变量和成员函数分开储存。

类的储存中只包含非静态成员变量，静态成员、非静态成员函数均在其它内存空间储存。

一种类的非静态成员函数之存在一个内存位置，不同类分别可调用该函数。

```c++
class Person(){
   
}; 
Person P;
cout << sizeof(P)<<endl ; // 返回一个字节（空对象占用一个字节以区分）
class Person(){
	int age;   
}; 
Person P;
cout << sizeof(P)<<endl ; // 返回4个字节（非静态成员变量占用空间）
class Person(){
   int age;
   int fun(){}
}; 
Person P;
cout << sizeof(P)<<endl ; // 返回4个字节（非静态成员函数不在类中储存）

```

+ this指针

由于一种类的非静态成员函数都存为一个。因此为方便非静态成员函数区分不同的调用，使用this来表示被调用的成员函数所属对象的指针。

**链式编程**

```c++
	Student& getDay(Student & s) {
		this->age += s.age;
		return *this;
	}
	s1.getDay(s2).getDay(s2);

```

对于使用空指针初始化的类对象，也可以直接访问成员函数，但是如果成员函数使用this指针，则会发生崩溃，因次通常设置判断处理来避免崩溃。

this指针本质上是一个指针常量，指向对象本身。如果需要设置常量指针常量以阻止指针指向值的更改，则在形参列表后面加const.

```c++
class Student(){
public:
	int m_A;
    mutable inr m_B;
    // 定义一个常函数，常函数的const实际上修饰的是this指针，常函数内部不可修改属性的值。
	void showPerson() const
	{
        this = NULL; // 错误，不可更改this指向
		this->m_A = 100; // 报错，this被const修饰，成为常量指针常量，值不可以更改
        this->m_B = 200; //不报错，mutable修饰的属性在常函数中也可以修改
	}
}
```

**常对象的属性也不可以更改，但是mutable修饰的属性可以更改。常对象只能调用常函数。**

## 12.类的友元

+ 全局函数做友元

一般的全局函数无法访问类的私有变量，设置为友元函数后就可以访问。

```c++
class room{
    friend void fun2();
public:
	string sittingRoom;

private:
    string bedroom="bedroom";
}
void fun(){
    room r;
    cout<<r.redroom<<endl; //报错。不能访问私有属性
}
void fun2(){
    room r;
    cout<<r.redroom<<endl; //不报错。friend能访问私有属性
}
```

+ 类做友元

```c++
class room{
    friend gay;
public:
	string sittingRoom;

private:
    string bedroom="bedroom";
};

class gay{
public:
    room* r;
    void fun(){
        r=new room;
        cout<<r->bedroom<<endl; // 保持
    }
    
}
```

+ 类的成员函数作为另外一个类的友元

```c++
friend void ClassName::fun();
```

## 13.运算符重载

根据新的数据类型设置新的定义的符号含义。

内置的数据类型不可以发生运算符重载。

+ 加法运算符（类内、全局）

```c++
	Student operator+(Student &s) {
		Student temp;
		temp.age = this->age + s.age;
		return temp;
	}
```

+ 左移运算符（类内需要调换位置，内外可以正常）

```c++
class Student{
    void operator<<(ostream & cout) {
		cout << this->age;
		cout << this->name;
	}
    
}	
// 使用时cout在右边
S<<cout;
// 全局重载，使用时cout在左边，可以返回cout作为链式调用
ostream& operator<<(ostream& cout, Student& s) {
	cout << s.age;
	return cout;
}
```

+ 自加、自减运算符

```c++
Student& operator++(){

} // 前置自加，先加后用
Student& operator++(int){

}  // 使用占位符重载，后置自加，先用后加
```

+ 赋值运算符重载

**当对类进行拷贝调用时，有时由于浅拷贝的问题，使得程序崩溃。此时，需要用赋值运算符重载，在类的实现中建立新的类的赋值深拷贝。**

```c++
	Student& operator=(Student & s) {
		if (sex != NULL) {
			delete sex;
			sex = NULL;
		}
		sex = new int(*s.sex);
		return *this;
	}

	~Student() {
		if (sex != NULL){
			delete sex;
			sex = NULL;
		}
	}

```

使用时，直接a=b，不要写成类初始化的等号赋值。

+ 关系运算符
+ 函数调用运算符（仿函数）

```c++
class Student
{
	friend ostream& operator<<(ostream& cout, Student& s);
private:
	int  score=4;
	//int* sex;
public:

	int operator()(int score) {
		if (this->score > score) {
			return 0;
		}
		else return 1;
	}

}; 
// 使用举例：非常像直接调用函数，因此称为仿函数
int a = s(2);
```

匿名函数对象：

```c++
// 只想使用类的重载运算符，使用完后释放内存，而不保存，可以使用Student()声明一个匿名对象，并使用功能。
cout<< Student()(100)<<endl
```

## 14.继承

类的继承包含三种方式：

```c++
class son:public/private/protected father{

};
```

public:继承父类的公共、保护的非静态成员变量，权限不变；

protected:以保护的权限继承父类的公共、保护的非静态成员变量；

private:以私由的权限继承。

**继承时，无论以何总权限，都会继承所有的父类非静态成员变量（包括私由），只是访问的权限不同。**

使用工具查看文件内某个类的继承关系：

```bash
// 在vs工具->命令行->开发者命令提示中输入：
cl /d1 reportSingleClassLayout类名 类所在的文件
```

当子类中出现和父类同名的元素，会直接覆盖掉父类中的同名成员：

```c++
son.a :访问子类中的
son.father::a :访问父类中的
    
// 访问静态成员时，也可以使用下面用类名的方法访问：
son::a
// 第一个双冒号表示访问son中的元素father，第二个双冒号表示父类作用域下的元素a
son::father::a
```

多继承

```
class son:public father1,public father2{}

// 当father1、father2有相同的元素时，son必须添加作用域来访问需要的
```

菱形继承

+ 一个共同的父类派生了两个不同子类，而这两个子类又有一个共同的儿子，此时成为菱形继承。
+ 菱形继承的祖父类的属性，两个父亲类都有，而儿子类继承自两个父亲类，使得它出现多继承的问题，同时，它也具有那个属性
+ 利用虚继承可以解决菱形继承的问题。

```c++
class grandfather{
public:
	int age=60;
}
class father1:public grandafather{
    
}
class father2:public grandfather{}
class son:public father1,public father2{}
int main(){
    son s;
	s.father1::age = 20;
    s.father2::age = 30;

    cout << s.father1::age << endl;
	cout << s.father2::age << endl;
    cout<<s.age<<endl; //出现不明确的报错
    
    
    return 0;
}
// 在父类继承前加上virtual 声明为虚基类即可，显示类继承关系时显示未vbptr虚基类指针，系统没有建立两个不同的father,而是将相同的合在一起。
#include <iostream>
#include <string>
using namespace std;

class grandfather {
public:
    int age=60;
};
class father1 :virtual public grandfather {
public:
	string name = "father1";
};
class father2 :virtual public grandfather {
public:
    string name = "father2";
};
class son :public father1, public father2 {};
int main() {
    father1 f1;
    father2 f2;
    son s;
	s.father1::age = 20;
    s.father2::age = 30;

    cout << s.father1::age << endl;
	cout << s.father2::name << endl;
	cout << s.age << endl;
    return 0;
}

```

![image-20241106204032590](https://gitee.com/airporal/image_hub/raw/master/img/202411062040725.png)

​	相同的虚基类继承部分变为一个虚基类指针vbptr，指向同一个数据，而不同部分则正常使用。

## 15.多态

+ 函数重载、运算符重载都属于多态
+ 继承中，子类重写父类函数的虚函数，成为动态多态

**用父类指针、引用指向不同的子类对象。**

```c++
#include <iostream>
#include <string>
using namespace std;

class Animal {
public:
    // 父类中声明虚函数，子类不需要再加virtual关键字
	virtual void speak() {
		cout << "Animal在说话" << endl;
	}
};

class Cat :public Animal {
public:
	void speak() {
		cout << "Cat在说话" << endl;
	}
};

void test01(Animal & animal) {
	// 调用重写的虚函数时，会根据使用的类来自定调用的是哪个函数重写
    animal.speak();

}
int main()
{
	Animal animal;
	Cat cat;
    // 父类引用指向子类对象，发生多态。
	test01(animal);
	test01(cat);
	return 0;
	
}
```

![image-20241107134226367](https://gitee.com/airporal/image_hub/raw/master/img/202411071342509.png)

virtual 会建立一个vfptr虚函数指针，指向一个虚函数表vftable，当使用该重写的方法时，会根据各个类找对应的虚函数指针，从而在虚函数表中找到对应重写的函数的位置。

**常用来做开放拓展接口**：

代码组织性强，可读性强，维护性强。

```c++
#include <iostream>
#include <string>
using namespace std;

class Calculator
{
public:
	int a, b;
	virtual int calculate() {
		return 0;
	}
};

class add :public Calculator {
public:
	int calculate() {
		return a + b;
	};
};

class sub :public Calculator {
public:
	int calculate() {
		return a - b;
	};
};

class Mix :public Calculator {
public:
	int calculate() {
		return a * b;
	};
};
void test(Calculator* c) {
	cout << c->calculate() << endl;
}

int main()
{
	Calculator* cal = new add();
	cal->a = 10;
	cal->b = 20;
	test(cal);
	delete cal;

	cal = new Mix();
	cal->a = 10;
	cal->b = 20;
	test(cal);
	delete cal;

	return 0;
	
}
```

+ 纯虚函数

如上面代码中Calculator中的calculate函数，在父类中基本没有意义，只是为了方便后续多态实现而设置，此时为了方便也可以设置为纯虚函数。

含有纯虚函数的类为抽象类，抽象类无法实例化对象，子类必须重写父类中的纯虚函数，否则也是抽象类，也无法实例化对象。

```c++
#include <iostream>
#include <string>
using namespace std;

class AbstractDrinking {
public:
	virtual void Boil() = 0;
	virtual void Brew() = 0;
	virtual void PourInCup() = 0;
	virtual void PutSomething() = 0;
};
class Coffee : public AbstractDrinking {
public:
	virtual void Boil() {
		cout << "Boil water" << endl;
	}
	virtual void Brew() {
		cout << "Brew coffee" << endl;
	}
	virtual void PourInCup() {
		cout << "Pour coffee into cup" << endl;
	}
	virtual void PutSomething() {
		cout << "Put sugar and milk" << endl;
	}

};
class Tea : public AbstractDrinking {
public:
	virtual void Boil() {
		cout << "Boil water" << endl;
	}
	virtual void Brew() {
		cout << "Brew tea" << endl;
	}
	virtual void PourInCup() {
		cout << "Pour tea into cup" << endl;
	}
	virtual void PutSomething() {
		cout << "Put lemon" << endl;
	}


};
void doWork(AbstractDrinking* abs) {
	abs->Boil();
	abs->Brew();
	abs->PourInCup();
	abs->PutSomething();
	delete abs;
}
void test01() {
	doWork(new Coffee);
	cout << "-------------------" << endl;
	doWork(new Tea);
}
int main()
{
	test01();
	return 0;	
}
```

释放内存时，继承的类不会调用子类的析构函数。当子类中有堆内数据时，由于不会调用子类的析构函数，使得子类无法直接释放堆内数据，此时需要使用虚析构函数释放子类的堆上数据。

```c++
// 父类中声明虚析构函数	
virtual ~AbstractDrinking() {
		cout << "AbstractDrinking destructor" << endl;
	}

// 子类写自己的虚构函数可以正常使用
    ~Coffee() {
    delete sell;
    cout << "Coffee destructor" << endl;
}
// 或者纯虚析构函数，但必须在类外实现。
class AbstractDrinking {
public:
	virtual void Boil() = 0;
	virtual void Brew() = 0;
	virtual void PourInCup() = 0;
	virtual void PutSomething() = 0;
	virtual ~AbstractDrinking() = 0;
};
AbstractDrinking::~AbstractDrinking() {
	cout << "AbstractDrinking destructor" << endl;
}
```

## 16 文本操作

三种文件读写流类：ifstream（只读） ofstream（只写） fstream（写读）。  

头文件fstream

文件分为两种，为本文件和二进制文件，文本文件以ASCII码的形式储存。

读文件：文本文件最后都以EOF结尾

1. 读文本文件

```c++
		ofstream ofs;
	ifstream ifs;
	ofs.open("MyFile.txt", ios::out|ios::app);
	if (ofs.is_open()) {
		ofs << "Hello world" << endl;
	}
	ofs.close();
	/*
	打开方式：
	ios::in		打开文件用于读取
	ios::out	打开文件用于写入
	ios::app	追加方式打开文件
	ios::ate	打开文件并定位到文件尾
	ios::trunc	如果文件已经存在，先删除文件
	ios::binary	以二进制方式打开文件
	*/
	ifs.open("MyFile.txt", ios::in);
	if (!ifs.is_open()) {
		
	}
	else {
        // ifs到字符串，一次读一行，ifs到字节，一次读一个
		char buf[1024] = { 0 };
		while (ifs>>buf) {
			cout << buf << endl;
		}
        //或者
       	while (ifs.getline(buf,sizeof(buf))) {
			cout << buf << endl;
		}
        // 或者
    	string buf;
		while (getline(ifs,buf)) {
			cout << buf << endl;
		}
	}
```

2. 写二进制文件

```c++
/*
首先以二进制的方法打开文件
编写需要的文件内容，不要使用string,要使用char *,因为底层不是c++
使用write写入，并强制转换为const char*
*/
```

3. 读写二进制文件

```c++
#include <iostream>
using namespace std;
#include <fstream>
#include <string>

class Student {
public:
	int age;
	char name[64];
};

int main()
{	
	Student p = { 18,"张三" };
	ofstream ofs("Student.txt", ios::out | ios::binary);
	//ofs.open("Student.txt", ios::out|ios::binary);
	if (ofs.is_open()) {
		ofs.write((const char*)&p, sizeof(Student));
	}
	ofs.close();

	ifstream ifs("Student.txt", ios::in | ios::binary);
	if (ifs.is_open()) {
		Student p;
		ifs.read((char*)&p, sizeof(Student));
		cout << "姓名：" << p.name << " 年龄：" << p.age << endl;
	}

	/*
	打开方式：
	ios::in		打开文件用于读取
	ios::out	打开文件用于写入
	ios::app	追加方式打开文件
	ios::ate	打开文件并定位到文件尾
	ios::trunc	如果文件已经存在，先删除文件
	ios::binary	以二进制方式打开文件
	*/

	return 0;	
}
```

## 17 模版

+ 函数模版

template<typename T>或template<class T>
函数定义 （数据类型可被T替代）
例如：

```c++
template<typename T>
T add(T a, T b)
{
	return a;
}
int main()
{
	cout << add('a', 'b') << endl;
    cout << add<char>('a', 'b') << endl;
	return 0;
}
```

函数模版下面需要接上函数定义，定义需要使用该模版。或在调用时指定类型。

普通函数调用时，可以进行自动的类型转换，但是模版函数必须指定类型时才能自动类型转换。

函数模版和普通函数重载时，优先调用普通函数。

有进行类型匹配时，优先调用函数模版；有<>时调用类型模版。

+ 类模版

类模版和函数模版不同点：

类模版不具有自动类型推导功能，类模版可以设置默认参数类型。

```c++
#include<iostream>
using namespace std;

// 类模版
template<class T, class ageName = int>
class Person {
public:
	T name;
	ageName age;
	Person(T name, ageName age);
	void display() {
		cout << "Name: " << name << " Age: " << age << endl;
        // 查看数据类型
		cout << "Type of name: " << typeid(name).name() << endl;
		cout << "Type of age: " << typeid(ageName).name() << endl;
	}
};


// 构造函数的类外实现
template<class T, class ageName = int>
Person<T,ageName>::Person(T name, ageName age) {
		this->name = name;
		this->age = age;
}


// 类模版作为函数基类被继承时，子类需要指定模版类型才可继承，可直接传入具体类型，也可以再次用模版代替。
template<class T, class ageName = int>
class Person2 : public Person<T, ageName>{
public:
	void printType() {
		cout << "Type of name: " << typeid(name).name() << endl;
		cout << "Type of age: " << typeid(age).name() << endl;
	}
    void fun()
};

// 类模版对象作为函数参数的三种方法，第一种最常用
void test01(Person<string, int>& p) {
	p.display();

}
template<class T>
void test02(Person<T>& p) {
	p.display();
}
template<class T>
void test03(T& p) {
	p.display();
}

int main() {
	Person<string, int> p1("John", 25);
	test01(p1);
	Person<string> p2("Doe", 30);
	test02(p2);
	Person<string> p3("Doe", 30);
	test03(p3);
	return 0;
}
```

类模版的成员函数在实例确定模版类型的时候才创建。

分文件编写时，由于类模版成员函数一开始没有建立，在cpp源文件中没有对头文件中成员函数的链接，调用时，会出现无法解析外部命令的问题：

方法1：直接包含cpp文件

方法2：类模版使用hpp文件编写在一个文件中（推荐）

```c++
// 类模版的友元函数
#include<iostream>
#include <string>
using namespace std;
class Person;
template<class T, class ageName = int>
void printType2(Person<T, ageName> p) {
		cout << "Type of name: " << typeid(p.name).name() << endl;
		cout << "Type of age: " << typeid(p.age).name() << endl;
	}   

template<class T, class ageName = int>
class Person {
    // 全局函数，友元内类实现
	friend void printType1(Person<T, ageName> p) {
		cout << "Type of name: " << typeid(p.name).name() << endl;
		cout << "Type of age: " << typeid(p.age).name() << endl;
	}
    friend void printType2<>(Person<T, ageName> p)
public:
	Person(T name, ageName age) {
		this->name = name;
		this->age = age;
	}
private:
   	T name;
	ageName age;
};

int main() {
	Person<string> p3("Doe", 30);
	printType4(p3);
	return 0;
}
```

全局函数作为类模版的友元，如果在内外实现，需要先声明类，再定义全局函数，再在类内设置声明为友元函数。

## 18. STL

> 为提高算法复用性，设计了标准的函数容器、算法、迭代器、类函数

+ 容器：存放数据
+ 算法：操作容器中的数据
+ 迭代器：使得算法可以方位容器中的数据，类似指针。（双向迭代器、随机迭代器）



### 18.1 string 

+ 构造函数

```c++
#include <string>
using namespace std;  // 指定缺省的使名空间。
string str;       // 创建string对象。
str = "data";	//赋值
// 迭代器
string::iterator begin = str.begin();
string::iterator end = str.end();
// 插入
int len = str.length(); //返回字符串长度
str.insert(n,"new");
str.pushback;
// string 转int
//atoi（）可以将char * 的字符串转为数字，先使用string的c_str()属性转为char *
int a = atoi(str.c_str());
```

### 18.2 vector

```c++
#include <vector>
#include <algorithm>
using namespace std;
// 构造函数
vector<int> name; //实例化一个空的整形数组
vector<int> name(7); //实例化一个包含7个数的整形数组，每个都默认为0
vector<int> name(7,2); //实例化一个包含7个数的整形数组，每个都是2
vector<int> name = {1,2,3,4};// 初始化一个包含1,2,3,4的数组
// name是数组的头指针

//向数组中添加内容
name.push_back(4); //将4加到数组末尾
//索引数组第n个元素
int val = name[n];
// 迭代器
name.begin();
name.end();
//获取数组大小
int size = name.size();
// 遍历数组
for (auto it = myVector.begin(); it != myVector.end(); ++it) {
    std::cout << *it << " ";
}

//或者（必须以此形式声明int，否则可能报错）
for (int element : myVector) {
    std::cout << element << " ";
}

// 判断是否为空
name.empty();
// 获取容量
name.capacity();
// 重设容量,大于原来的容量则会用默认值或者指定值填充，小于原来容量则删除
name.resize(15);
name.resize(15,0);

//根据地址删除元素
name.erase(name.begin()+2); // 删除第三个元素,传入迭代器
name.pop_back();
// 指定位置插入数据
name.insert(interator,element);

//清空数组
name.clear();
name.erase(iterator1,iterator2);// 删除迭代器之间的元素
//利用algorithm提供的方法可以找到向量最值下标
auto max_idx = max_element(name.begin(),name.end())
// 返回最大值
auto max = max_element(name.begin(),name.end());

// 交换容器内容,收缩内存
int main() {
	vector<int> v1;
	vector<int> v2;
    // 为v2预留空间，防止不必要的内存重新分配导致资源消耗
	v2.reserve(10000);
	int* p1 = NULL;
	int* p2 = NULL;
	int n1 = 0;
	int n2 = 0;
	for (int i = 0; i < 10000;i++) {
		v1.push_back(i);
		v2.push_back(i);
		if (p1!=&v1[0]) {
			p1 = &v1[0];
			n1++;
		}
		if (p2 != &v2[0]) {
			p2 = &v2[0];
			n2++;
		}

	}
    // v1总共分配了24次内存，v2只分配了一次。
	cout << n1 << endl;
	cout << n2 << endl;
	for (int i = 0; i < 10000;i++) {
		v1.push_back(i);

	}
	cout << v1.capacity() << endl;
	v1.resize(3);
	cout << v1.capacity() << endl;
	v2.swap(v1);
	printVector(v2);
	cout << v1.capacity() << endl;
	v1.resize(3);
    // 通过建立匿名对象拷贝有大量未使用空间的v1中的内容，并压缩空间
	vector<int>(v1).swap(v1);
	cout << v1.capacity() << endl;
	return 0;
}
```

### 18.3 deque

几乎和vector一样，但是内部有对数据的维护，不存在容量的概念，可以从前端和后端进行操作元素。

### 18.4 stack

栈是先进后出的数据结构，不可以遍历。

### 18.5 queue

队列是一种先入先出的数据结构，不可以遍历。

### 18.6 list

即链表。链表可以对任意的位置快速插入或者删除元素。相比于vector来说不需要移动其它的元素，在插入或删除过程中可以节省大量时间。

链表的储存地址不是在一块，不能通过迭代器的偏移来遍历，遍历的速度比较慢，占用的空间比较大。

不支持随机访问迭代器的容器，不可以使用标准算法库algorithm的函数。但会提供成员函数的方法。

```c++
// 构造函数
	list<int> L1;
	L1.push_back(1);
	L1.push_back(2);
	L1.push_front(3);
	L1.push_front(4);
	printList(L1);

	list<int> L2(L1);
	printList(L2);
	list<int>L3(++L1.begin(), L1.end());

// 赋值
L1.assig()
// 交换
    L1.swap()
// 改变大小
    L1.resize()
// 插入
    L1.insert(迭代器，值)
    // 尾插
    L1.push_back()
    // 头插
    L1.push_front();
// 删除
	L1.erase();
	L1.pop_front();
	L1.pop_back();
	L1.remove();
	L1.clear();
// 排序,默认降序，可以自行设置排序函数类似下面：
void funName(int a,int b){
    
    
}
	L1.sort(funName);
// 翻转
	L1.reverse()
```

### 18.7 set

set集合容器，在插入后会自动排序，不允许插入相同的值。

```c++
// 插入
set<int> s1;
s1.insert(2);
// 删除
s1.clear();
s1.erase();
// 自定义排序规则（自定义类型时必须有）
class MyCompare {
public:
    bool operator()(const int& a, const int &b) const{
        return a > b;
    }
};
std::set<int, MyCompare> s1;
    for (set<int,MyCompare>::iterator it = s1.begin(); it != s1.end(); it++) {
        std::cout << *it << " ";
    }
    std::cout << std::endl;
```

multiset可以插入重复的数据。

```c++
multiset<int> s2;
```

对组pair

```c++
	pair<string,int>p("Airporal",22);
	cout << p.first << " " << p.second << endl;
	pair<string, int>p2 = make_pair("Airporal", 222);
	cout << p2.first << " " << p2.second << endl;
```



### 18.8 map

map容器中每个元素都是对组，第一个值是键key，第二个值是值value。

## 19.仿函数

### 19.1 仿函数用法

仿函数即是对类的括号运算符的重载。

```c++
#include <iostream>
#include <string>
#include <vector>
using namespace std;

class Fun {
public:
	void operator()(string str) {
		cout << "Calling Fun object with parameter: " << str << endl;
	}
};


int main() {
	Fun f;
	f("Hello");
	return 0;
}
```

### 19.2 谓语

返回bool类型的仿函数称为谓语。接收一个参数称为一元谓语，接收两个参数为二元谓语。谓语通常用作某些函数的传入参数（标识符号为Pr）。

```c++
#include <iostream>
#include <string>
#include <vector>
using namespace std;

class Fun {
public:
	bool operator()(int a) {
		//cout << "Calling Fun object with parameter: " << str << endl;
		return a > 4;
	}
};


int main() {
	vector<int> v = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
	vector<int>::iterator it = find_if(v.begin(), v.end(), Fun());
	if (it != v.end()) {
		cout << "Found: " << *it << endl;
	}
	else {
		cout << "Not found" << endl;
	}
	return 0;
}
```

```c++
// 二元谓语
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
using namespace std;

class Fun {
public:
	bool operator()(int a,int b) {
		//cout << "Calling Fun object with parameter: " << str << endl;
		return a > b;
	}
};

class Fun2 {
public:
	bool operator()(int a, int b) {
		//cout << "Calling Fun object with parameter: " << str << endl;
		return a > b;
	}
};


int main() {
	vector<int> v = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };

	sort(v.begin(),v.end(),Fun2());
	for (vector<int>::iterator it = v.begin(); it != v.end();it++) {
		cout << *it << ' ';
	}
	cout << endl;

	return 0;
}
```

### 19.3 算术仿函数

c++内置的仿函数，使用模版构建，可直接使用。

```c++
#include <functional>
// 取反仿函数
函数对象：
negate<T> name;
cout<<name(30)<<endl;
-50
// 加法仿函数
函数对象
plus<int> p;
cout<<p(10,20)<<endl;
30
// 减法
    plus<>
// 乘法
    multiplies<>;
// 除法
	divides<>;
// 取模
	modulus<>;

```

### 19.4 关系仿函数

```c++
equal_to<> // 等于
not_equal_to<> // 不等于
greater<T> // 大于
greater_equal<T> // 大于等于
less<> // 小于
less_equal<> // 小于等于
```

## 20 常用算法

### 20.1 遍历

for_each

```c++
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
using namespace std;

void print01(int v) {
	cout << v << " ";
}
class print02 {
public:
	void operator()(int v) {
		cout << v << " ";
	}
};

void test01() {
	vector<int> v;
	for (int i = 0; i < 10; i++) {
		v.push_back(i);
	}
	cout << "---匿名函数" << endl;
	for_each(v.begin(), v.end(), [](int val) {cout << val << " "; });
	cout << "---普通函数" << endl;
	for_each(v.begin(), v.end(), print01);
	cout << "---仿函数" << endl;
	for_each(v.begin(), v.end(), print02());

}

int  main() {
	test01();
	return 0;
}
```

**transform**

```c++
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

class Solution {
public:
	int operator()(int v) {
		return v * 2;
	}
};

void test01() {
	vector<int> nums = { 1, 2, 3, 4, 5 };
	vector<int> res(10);
	for_each(nums.begin(), nums.end(), [](int v) {cout << v << ' '; });
	transform(nums.begin(), nums.end(), res.begin(), Solution());
	cout << endl;
	for_each(res.begin(), res.end(), [](int v) {cout << v << ' '; });

}

int main() {
	test01();
	return 0;
}
```



### 20.2 查找

**find**:查找指定元素，如果自定义数据类型需要重载双等号运算符。

```c++
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

class Person {
public:
	string m_name;
	int m_age;
	Person(string name,int age) {
		this->m_name = name;
		this->m_age = age;
	}
	bool operator==(const Person& p) {
		if (this->m_name == p.m_name && this->m_age == p.m_age)
		{
			return true;
		}
		return false;
	}

};

void test01() {
	vector<int> v1;
	for (int i = 0; i < 10; i++) {
		v1.push_back(i);
	}
	//cout << *find(v1.begin(),v1.end(),13) << endl;
	vector<Person> v2;
	Person p1("aaa", 10);
	Person p2("bbb", 20);
	Person p3("ccc", 30);
	Person p4("ddd", 40);
	v2.push_back(p1);
	v2.push_back(p2);
	v2.push_back(p3);

	cout << find(v2.begin(), v2.end(), p1)->m_name << endl;
}

int main() {
	test01();
	return 0;
}
```

**find_if**：按照指定规则查找第一个满足条件的元素的迭代器

```c++
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

class Person {
public:
	string m_name;
	int m_age;
	Person(string name,int age) {
		this->m_name = name;
		this->m_age = age;
	}
	bool operator==(const Person& p) {
		if (this->m_name == p.m_name && this->m_age == p.m_age)
		{
			return true;
		}
		return false;
	}

};
class findPerson {
public:
	bool operator()(const Person& p) {
		if (p.m_age == 20) {
			return true;
		}
		return false;
	}
};

void test01() {
	vector<int> v1;
	for (int i = 0; i < 10; i++) {
		v1.push_back(i);
	}
	//cout << *find(v1.begin(),v1.end(),13) << endl;
	vector<Person> v2;
	Person p1("aaa", 10);
	Person p2("bbb", 20);
	Person p3("ccc", 30);
	Person p4("ddd", 40);
	v2.push_back(p1);
	v2.push_back(p2);
	v2.push_back(p3);

	cout << find(v2.begin(), v2.end(), p1)->m_name << endl;
	vector<Person>::iterator it = find_if(v2.begin(), v2.end(), findPerson());
	if (it!=v2.end()) {
		cout << "find it" << it->m_name<< endl;
	}
	else {
		cout << "not find it" << endl;

	}
}

int main() {
	test01();
	return 0;
}
```

**adjacent_find**：查找第一个相连的相同元素

```c++
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

class Person {
public:
	string m_name;
	int m_age;
	Person(string name,int age) {
		this->m_name = name;
		this->m_age = age;
	}
	bool operator==(const Person& p) {
		if (this->m_name == p.m_name && this->m_age == p.m_age)
		{
			return true;
		}
		return false;
	}

};
class findPerson {
public:
	bool operator()(const Person& p) {
		if (p.m_age == 20) {
			return true;
		}
		return false;
	}
};

void test01() {
	vector<Person> v2;
	Person p1("aaa", 10);
	Person p2("bbb", 20);
	Person p3("ccc", 30);
	Person p4("ddd", 40);
	v2.push_back(p1);
	v2.push_back(p2);
	v2.push_back(Person("bbb", 20));
	v2.push_back(p3);
	v2.push_back(Person("ccc",20));

	vector<Person>::iterator ir = adjacent_find(v2.begin(),v2.end());
	cout << ir->m_name << endl;
}

int main() {
	test01();
	return 0;
}
```

**binary_search**：对有序序列进行高效的二分查找。

**count**：统计某个数据出现的次数

```c++
cout << count(v2.begin(), v2.end(), p2) << endl;
```

**count_if**：按照谓语查找满足的值的个数。

### 20.3 排序

**sort**：默认从小到大排序，可以设置谓语调整排序方法

```c++
class fun{
    public:
    bool operator()(int a, int b){
        return a>b;
    }
    
};

vector<int> v;
v.push_back(3);
v.push_back(23);
v.push_back(13);
v.push_back(33);
sort(v.begin(),v.end());
for_each(v.begin(),v.end(),[](int v){cout<<v<<' ';1});
// 3 13 23 33
sort(v.begin(),v.end(),fun());
for_each(v.begin(),v.end(),[](int v){cout<<v<<' ';1});
// 33 23 13 3
```

**random_shuffle**：洗牌

```c++
	random_shuffle(v2.begin(), v2.end());
	for_each(v2.begin(), v2.end(), [](Person p) {cout << p.m_name << " " << p.m_age << endl; });
```

**merge**：将两个有序序列合并成一个新的有序序列

**reverse**：将容器翻转

### 20.4 拷贝与替换

**copy：** 利用copy算法将一个容器的部分数据拷贝到另外一个容器，先要给该容器开辟空间。

**replace**：将某个值替换为另外的值

**replace**：按照谓语将值替换为指定的值。

**swap**：交换两个容器。

```c++
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

class MyFunctor {
public:
	bool operator()(int a) {
		return a > 3;
	}
};

int main() {
	vector<int> v;
	v.push_back(1);
	v.push_back(2);
	v.push_back(3);
	v.push_back(4);
	v.push_back(5);
	v.push_back(3);

	vector<int> v2;
	v2.resize(v.size());
	copy(v.begin(), v.end(), v2.begin());
	for_each(v2.begin(), v2.end(), [](int i) {cout << i << " "; });
	cout << endl;	
	replace(v2.begin(), v2.end(), 3, 300);
	for_each(v2.begin(), v2.end(), [](int i) {cout << i << " "; });
	cout << endl;
	replace_if(v2.begin(), v2.end(),MyFunctor(), 3);
	for_each(v2.begin(), v2.end(), [](int i) {cout << i << " "; });
	cout << endl;
	swap(v,v2);
	for_each(v2.begin(), v2.end(), [](int i) {cout << i << " "; });
	system("pause");
	return 0;
}
```

### 20.5 其它

 **accumulate**：累加函数。

**fill**：将容器指定区间中的值填充为指定的值。

**set_intersection**：当两个原容器是有序序列时，可以求出两个容器中交集部分并复制给新的目标容器，返回目标容器中交集数据的最后一位迭代器。需要先给目标容器开拓空间。

**set_union**：获取有序序列的并集。

**set_differerce**：取前一个容器与后一个容器的差集合，即前一个容器中有后一个容器中没有的元素的集合。

```c++
#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
using namespace std;

class MyFunctor {
public:
	bool operator()(int a) {
		return a > 3;
	}
};

int main() {
	vector<int> v;
	v.push_back(1);
	v.push_back(2);
	v.push_back(3);
	v.push_back(4);
	v.push_back(5);
	v.push_back(9);
	int n = accumulate(v.begin(), v.end(), 1);
	cout << n << endl;
	v.resize(10);
	cout << v.capacity() << endl;
	for_each(v.begin(), v.end(), [](int a) {cout << a << " "; });
	cout << endl;
	fill(v.begin()+6, v.end(), 10);
	for_each(v.begin(), v.end(), [](int a) {cout << a << " "; });
	cout << endl;
	
	vector<int> v2 = {1,2,4,6,8};
	vector<int> vTarget;
	vTarget.resize(min(v2.size(),v.size()));
	vector<int>::iterator endIt =  set_intersection(v.begin(), v.end(), v2.begin(), v2.end(), vTarget.begin());
	for_each(vTarget.begin(), endIt, [](int a) {cout << a << " "; });
	cout << endl;

	vector<int> v3 = { 1,2,4,6,8 };
	vector<int> vTarget2;
	vTarget2.resize(v3.size()+v.size());
	vector<int>::iterator endIt2 = set_union(v.begin(), v.end(), v3.begin(), v3.end(), vTarget2.begin());
	for_each(vTarget2.begin(), endIt2, [](int a) {cout << a << " "; });
	cout << endl;

	vector<int> v4 = { 1,2,4,6,8 };	
	vector<int> vTarget3;
	vTarget3.resize(max(v.size(),v4.size()));
	vector<int>::iterator endIt3 = set_difference(v.begin(), v.end(), v4.begin(), v4.end(), vTarget3.begin());
	for_each(vTarget3.begin(), endIt3, [](int a) {cout << a << " "; });
	cout << endl;

	vector<int> vTarget4;
	vTarget4.resize(max(v.size(), v4.size()));
	vector<int>::iterator endIt4 = set_difference(v4.begin(), v4.end(), v.begin(), v.end(), vTarget4.begin());
	for_each(vTarget4.begin(), endIt4, [](int a) {cout << a << " "; });

	system("pause");
	return 0;
}
```



# 二. QT基础

## 1. QT初始化约定

QT自动生成的主程序main.cpp:

```c++
#include "Qt_learn1.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Qt_learn1 w;
    w.show();
    return a.exec();
}

```

​	导入自定义名称的类头文件，并创建程序main函数，main函数中创建整个程序唯一的应用程序对象；创建自定义窗口对象的实例。

​	窗口对象默认不会显示，使用show()方法显示。最后进入应用程序的消息循环，监听用户行为并执行处理逻辑。

Qt_learn1.h文件：

```cpp
#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_Qt_learn1.h"

class Qt_learn1 : public QMainWindow
{
    Q_OBJECT // Q_OBJECT宏，运行类中使用信号和槽的机制

public:
    Qt_learn1(QWidget *parent = nullptr); // 构造函数，并传递默认参数为空指针，在Qt_learn1.cpp文件中定义。
    ~Qt_learn1();  //析构函数

private:
    Ui::Qt_learn1Class ui;
};

```

Qt_learn1.cpp文件：

```c++
#include "Qt_learn1.h"

Qt_learn1::Qt_learn1(QWidget *parent)
    : QMainWindow(parent) // 调用父类进行初始化
{
    ui.setupUi(this);
}

Qt_learn1::~Qt_learn1()
{}

```

一般在此文件中编写UI界面。

## 2. QT Button

初始化一个button:

```c++
// 实例化一个Button,new用来动态分配内存，加上构造函数即可初始化对象并返回地址
QPushButton * button = new QPushButton;
// 使用->符号调用实例化的对象的方法，下面显示该按钮
button->show();
```

仅仅这样初始化后，会在新窗口中生成一个没有任何特征的button。

![image-20240927144638750](https://gitee.com/airporal/image_hub/raw/master/img/202409271446796.png)

为了将button附着在主窗口上，需要设置它的父组件为当前对象：此时不再需要show

```c++
button->setParent(this);
```

![image-20240927144935194](https://gitee.com/airporal/image_hub/raw/master/img/202409271449241.png)

给按钮加上文字，中文会出现乱码。

```c++
button->setText("Button1")
```

![image-20240927145437004](https://gitee.com/airporal/image_hub/raw/master/img/202409271454051.png)

也可直接在初始化时定义父组件和内容

```c++
QPushButton* button2 = new QPushButton("Button2", this);
```

其它方法：

```c++
resize(w,h); //设置窗口尺寸
setWindowTitle("name"); //设置窗口名
setFixedSize(w,h);//设置窗口固定尺寸
button->resize(w,h); //设置按钮尺寸
```

## 3. QT 对象树

QT的对象树在一定程度上简化了内存回收机制，新建一个组件对象后，如果指定了该对象的父亲类，且父亲类是QT提供的类或任何已经在对象树中的类，都会自动将新对象加入到对象树中。

在释放窗口时，会按照对象树从下到上依此释放各个对象，并运行自定的析构函数和默认的析构函数。

==**Q_OBJECT宏：**==启用Qt中元对象系统，必须要放在任何需要使用信号和槽机制、动态属性、或其它元对象功能的类的声明中。

QT的坐标系：左上角是（0,0）。

## 4. 信号和槽

对于每个组件，可以监听用户鼠标的操作信号，在QT assistance 中可以查看各个组件支持的信号，其父类的信号各个组件也可用。

![image-20240928164245233](https://gitee.com/airporal/image_hub/raw/master/img/202409281642353.png)

**使用connect函数传递信号：**

connect(信号发送者指针，发送的Signal函数指针，信号的接受者指针，信号处理槽函数指针)

```c++

connect(button2, &MyPushButton::clicked, this, &Qt_learn1::lower);
connect(button, &QPushButton::clicked, button2, &QPushButton::click);
```

参数2、4本质上就是两个函数指针即可，参数1和3分别是接受信号的组件和执行动作的组件

**自定义信号和槽：**

信号：头文件中，在signals:后面添加信号，要求：无返回值，只需要声明不需要实现，可以有参数可以有重载。

```c++
// 在h文件下组件类下声明
signals:
	void hungry();
```

槽函数：早期版本必须在public slots下，5.6以上版本可以写在public或者全局下。要求：无返回值，需要声明（h下）、需要实现（cpp下）、可以有参数、可以有重装，

```c++
// 在h文件下组件类下声明
public slots: //处理信号的槽用
    void treat();
// 在cpp文件中定义
void Student::treat() {
	qDebug() << "play game";
}
```

在主widget文件的h文件中声明各个组件的实例，在cpp文件中执行逻辑：

**MyWidget.h文件：**

```c++
#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_MyWidget.h"
#include "Student.h"
#include "Teacher.h"

class MyWidget : public QMainWindow
{
    Q_OBJECT

public:
    MyWidget(QWidget *parent = nullptr);
    ~MyWidget();


private:
    Ui::MyWidgetClass ui;
	// 声明老师和学生对象
	Teacher* teacher;
	Student* student;
    void classIsOver();
};

```

MyWidget.cpp文件

```c++
#include "MyWidget.h"

MyWidget::MyWidget(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	// 创建老师和学生对象
	this->teacher = new Teacher(this);
	this->student = new Student(this);
    // 设置信号连接，函数未进行复用时可直接写函数指针
	connect(teacher, &Teacher::hungry, student, &Student::treat);
    // 模拟下课事件
	classIsOver();

}
//设置一个事件模拟的函数，调用该函数会
void MyWidget::classIsOver() {
	// 老师饿了信号。emit关键字发射该信号
	emit teacher->hungry();
}

MyWidget::~MyWidget()
{}

```

**复用信号或者槽函数：**

复用信号时，只需要在信号声明的对象组件处声明复用的内容，使用的时候需要使用定义新的函数指针指定该复用的函数。

```c++
signals://声明信号用
	void hungry();
	void hungry(QString foodName);

public slots: //处理信号的槽用
};

```

复用槽函数时，需要再声明并定义新的复用内容，使用的时候同样需要用新定义的函数指针指向要用的槽函数。

```c++
signals://声明信号用

public slots: //处理信号的槽用
    void treat();
	void treat(QString foodName);
};
```

```c++
}
void Student::treat() {
	qDebug() << "I want to eat!";
}
void Student::treat(QString foodName) {
	qDebug() << "I want to eat " << foodName;
    // QString 输出时会带引号，需要转化为QByteArray再转为char * 就可以不带引号输出。
    qDebug()<<"I want to eat"<<foodname.toUtf8().data()
}
```

![image-20240930161717779](https://gitee.com/airporal/image_hub/raw/master/img/202409301617865.png)

MyWidget.cpp文件：

```c++
#include "MyWidget.h"


MyWidget::MyWidget(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	// 创建老师和学生对象
	this->teacher = new Teacher(this);
	this->student = new Student(this);
   // 新建函数指针，并指定参数、指向的函数。根据参数来区分不同复用的重名函数。
	void(Teacher:: * tearcherSignal)(QString) = &Teacher::hungry;
	void(Student:: * studentSlot)(QString) = &Student::treat;
	connect(teacher, tearcherSignal, student, studentSlot);
	classIsOver("chicken");

}
void MyWidget::classIsOver(QString meal) {
	// 老师饿了
	emit teacher->hungry(meal);
}

MyWidget::~MyWidget()
{}

```

> ​	信号函数定义实际是定义信号名称和信号格式，connect后与另外的组件关联，满足信号格式的内容将传递到**下一个或多个组件**，并以该信号内容作为参数，触发绑定的组件的处理槽函数。**槽函数也可以是信号。信号也可以连接多个槽函数**。信号的个数可以多余槽函数参数的个数，但是信号类型必须对应。

**使用disconnect可以断开既定的连接。参数同connect。**

**Lambda表达式做槽函数：**

```c++
connect(button, &QPushButton::clicked, this, [=]() {
		emit teacher->hungry("fish");
		});
```

使用Lambda作为槽函数，【】:标识符，“=”表示值传递，“&”表示引用传递；（）表示参数；{}表示函数实现主体。若要返回值，则需要加上->返回值的类型：

```c++
connect(button, &QPushButton::clicked, this, [=]()->int {
		emit teacher->hungry("fish");
		});
// mutable 关键字：如果要修改匿名函数接受的参数的值就需要使用mutable关键字
connect(button,&QPushButton::ckicked,this,[m]()->int mutable{m=10;emit teacher->hungry("fish")});
```

connect中，槽函数对应的组件可以忽略不写，即第三个参数可以忽略

## 5. QMainWindow

> QMainWindow下有四个子部件，菜单栏MenuBar、工具栏ToolBar、

5.1 菜单栏

+ 菜单栏函数包含头文件：Qmenubar.h，在新建菜单栏对象时会自动添加到对象树中。一个窗口菜单栏最多只有一个。

```c++
#include <qmenubar.h>
// 创建 menu bar 菜单栏
QMenuBar * my_menu = menuBar();
// 将菜单放到窗口上
setMenuBar(my_menu);

// 创建菜单
QMenu * fileMenu = my_menu->addMenu("File");
QMenu* editMenu = my_menu->addMenu("Edit");
// 创建菜单项
QAction * newFile = fileMenu->addAction("New");
QAction* inputFile = fileMenu->addAction("Input");
// 添加分割线
fileMenu->addSeparator();
fileMenu->addAction("Exit");
```

5.2 工具栏

+ 工具栏函数包含头文件：Qtoolbar.h，工具栏可以设置浮动或固定的窗口，可以添加控件或工具项。

```c++
#include <qtoolbar.h>
#include <qpushbutton.h>

// 创建工具栏
QToolBar * my_tool =  new QToolBar();
// 将工具栏放到窗口上,并设置停靠区域，停靠区域为Qt枚举类型，可以查手册选
addToolBar(Qt::LeftToolBarArea,my_tool);
// 设置可以停靠的区域 ，使用或运算符
my_tool->setAllowedAreas(Qt::LeftToolBarArea | Qt::RightToolBarArea);
my_tool->setMovable(0);

// 工具栏添加工具
my_tool->addAction(inputFile);
my_tool->addSeparator();
my_tool->addAction(newFile);
// 工具栏添加控件
QPushButton* my_button = new QPushButton("MyButton");
//my_button->setParent(my_tool);
my_tool->addWidget(my_button);
```

5.3 状态栏

+ 状态栏函数包含头文件：QstatusBar.h，可以设置为浮动或固定窗口。

```c++
#include <qstatusbar.h>
// 向状态栏中添加label组件
#include <qlabel.h>
// 创建状态栏
QStatusBar * my_status = statusBar();
// 加入到窗口中
setStatusBar(my_status);
// 新建状态控件
QLabel* label1 = new QLabel("Infor", this);
// 放置在左边
my_status->addWidget(label1);
// 放置在右边
QLabel* label2 = new QLabel("Message", this);
my_status->addPermanentWidget(label2);
```

5.4 浮动窗口

+ 浮动窗口包含头文件：qdockwidge.h，可以添加多个浮动窗口。

```c++
#include <qdockwidget.h>
// 创建浮动窗口
QDockWidget * my_dock = new QDockWidget("Dock", this);
addDockWidget(Qt::BottomDockWidgetArea,my_dock);
```

5.5 中心部件

+ 中心部件不需要额外头文件即可添加设置。只能有一个中心部件。

```c++
#include <qtextedit.h>
// 设置中心部件，可以设置为记事本或其它组件
QTextEdit* my_text = new QTextEdit(this);
setCentralWidget(my_text);
```

**只能有一个的部件都直接调用setxxx来新建，可以有多个的部件都使用addxxx来添加。**

## 6. QtDesigner使用

​	在建立Qt Application时，选择使用ui工具后，生成的工程项目中会包括一个名为name.ui的文件，在文件夹中双击以打开该文件。

![image-20241021192849746](https://gitee.com/airporal/image_hub/raw/master/img/202410211928865.png)

在该工具中编辑窗口以及组件以满足需求。

+ 代码中使用：

编辑好的ui文件，保存后会自动生成一个ui_QtDesigner.h文件，该文件自动实现在QtDesigner中设计的ui界面，并将各个组件封装在以ui为命名空间的对象中。

```c++
// 文件位置
// ~/QtDesigner/x64/Debug/uic/ui_name.h
// 使用方法
// 导入头文件
#include "QtUiDesigner.h"
//#include "ui_QtUiDesigner.h"
#include <qpushbutton.h>
QtUiDesigner::QtUiDesigner(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	// ui命名空间下对QtDesigner中绘制的窗口部件进行操作，可添加图标
    // 绝对路径：方便，但是更换设备不能使用。
    ui.actionnew->setIcon(QIcon("F:/picture/source/R-C.png"));
    // 相对路径
    ui.actionnew->setIcon(QIcon(":/source/R-C.png"));
    ui.actionopen->setIcon(QIcon("icon1.ico"));

}

QtUiDesigner::~QtUiDesigner()
{}

```

​	上面在设计的ui界面的两个菜单按钮中添加了Icon图标，图表可以以绝对路径的方式添加，也可以加入到工程中，以相对路径的方式添加。

添加到工程中的方法：

1. 使用visual studio的资源方式添加

![image-20241021193908395](https://gitee.com/airporal/image_hub/raw/master/img/202410211939471.png)

依此点击上述按钮以打开添加资源；

![image-20241021194003833](https://gitee.com/airporal/image_hub/raw/master/img/202410211940896.png)

点击新建或者直接导入资源；

![image-20241021194455166](https://gitee.com/airporal/image_hub/raw/master/img/202410211944228.png)

完成后再资源管理器的Resoure Files栏目中出现对应的资源文件。使用时直接使用该名字即可。

```c++
    ui.actionopen->setIcon(QIcon("icon1.ico"));
```

2. 用Qt工具添加

依此点击下列按钮以新建项：

![image-20241021194624440](https://gitee.com/airporal/image_hub/raw/master/img/202410211946510.png)

选择添加Qt Resource File，并重命名。

![image-20241021194704591](https://gitee.com/airporal/image_hub/raw/master/img/202410211947659.png)

添加后，会新建一个res.qrc资源文件，仍然在Resource Files栏下。

添加时，先修改前缀Prefix，可直接设置为/

之和添加需要的资源文件，点击Add->Add Files即可。

![image-20241021195057491](https://gitee.com/airporal/image_hub/raw/master/img/202410211950562.png)

使用时，文件调用路径按照以下格式：

```c++
//  ：+prefix+相对路径，例如：
    ui.actionnew->setIcon(QIcon(":/source/R-C.png"));

```



## 7. 自定义控件

可以在vs中新建一个设计师ui文件，设计好自己的类然后作为一个新的组件使用。

+ 点击新建项

![image-20250103221612715](C:\Users\Administrator.DESKTOP-K7JDKF5\AppData\Roaming\Typora\typora-user-images\image-20250103221612715.png)

+ 新建一个Widgets class 并指定名称

![image-20250103221702760](C:\Users\Administrator.DESKTOP-K7JDKF5\AppData\Roaming\Typora\typora-user-images\image-20250103221702760.png)

+ 修改名称，并指定为Widget项目

![image-20250103221744501](C:\Users\Administrator.DESKTOP-K7JDKF5\AppData\Roaming\Typora\typora-user-images\image-20250103221744501.png)

+ 在新建的ui文件中完成设计并保存

![image-20250103222033177](https://cdn.jsdelivr.net/gh/Airporal/Pictures/img/202501032220359.png)

+ 在主窗口中新建一个Widget，并设置提升为该新建的自定义ui，最后保持即可

![image-20250103222218269](https://cdn.jsdelivr.net/gh/Airporal/Pictures/img/202501032222463.png)

## 8. 鼠标事件

​	鼠标在与组件交互过程中会自动产生一些信号，在QWdiget中已经声明好了，当需要使用这些鼠标事件的时候，需要重写这些虚函数。

+ 字符串格式化输出

```c++
QString str = QString("Mouse x = %1    Mouse y = %1").arg(event->position().x()).arg(event->position().y());
qDebug() << str;
```

+ void enterEvent(QEnterEvent* event)

当鼠标移动入控件时触发该事件，使用event可以捕获鼠标状态。

+ void mousePressEvent(QMouseEvent* event)

当鼠标点击时可以触发该事件，可以使用event指针查看点击的键和鼠标指针状态。

+ void mouseMoveEvent(QMouseEvent* event)

当按下鼠标键并且移动时触发该事件

+ setMouseTracking(true)

设置为true时，鼠标状态每次变化会自动按下键触发相应事件。

```c++
// 一般在构造函数中调用setMouseTracking(true)
myLabel::myLabel(QWidget *parent)
	: QLabel(parent)
{
	setMouseTracking(true);
}
```

以上四种典型类型的鼠标事件还包括其它相似的事件可以使用。可在QMouseEvent中使用。

## 9. 定时器事件

​	定时器事件可以在指定的时间后触发事件，可以通过两种方式实现定时器事件。

+ 方法1：重写定时器触发事件，并通过设置多个不同的timerId实现多个不同的定时器

首先启动定时器，并指定时间：

```cpp
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	// 启动定时器，每隔1000ms触发一次
	this->id1 = startTimer(1000);
	this->id2 = startTimer(2000);

}
```

然后编写定时器事件处理的虚函数

```cpp
void MainWindow::timerEvent(QTimerEvent* event) {
    static int num = 1;
    static int num2 = 1;
    if (event->timerId() == this->id1) {
        ui.label_2->setText(QString::number(num++));
    }
    else { ui.label_3->setText(QString::number(num2++)); }

}

```

+ 方法2：堆区新建一个定时器事件，通过调动启动与结束函数并指定信号与槽来实现需要的事件与处理机制。

```cpp
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);
	// 启动定时器，每隔1000ms触发一次
	this->id1 = startTimer(1000);
	this->id2 = startTimer(2000);

    QTimer* timer = new QTimer(this);
    timer->start(500);
    connect(timer, &QTimer::timeout, [=]() {
        static int num3 = 1;
        ui.label_4->setText(QString::number(num3++));        
        });

    connect(ui.stopButton, &QPushButton::clicked, [=]() {
        timer->stop();
        });
    connect(ui.startButton, &QPushButton::clicked, [=]() {
        timer->start(500);
        });
}
```

## 10. 事件分发器

​	系统补足到定义好的各类型事件后，需要事件分发器进行事件的管理，事件分发器接受一个QEvent类型的事件指针，返回bool类型表示是否需要处理该事件。

```cpp
bool event(QEvent *e)
```

​	如果返回true表示用户处理了该事件，不再自动分发，用户定义的其它事件处理函数将无法捕捉该事件，返回false表示自动分发。

```c++
bool myLabel::event(QEvent* e) {
	// 如果是鼠标按下事件，就拦截
	if (e->type() == QEvent::MouseButtonPress) {
		// 强制类型转换,否则无法调用QMouseEvent的方法
		QMouseEvent* event = static_cast<QMouseEvent*>(e);
		QString str2 = QString("Mouse x = %1    Mouse y = %1").arg(event->globalPosition().x()).arg(event->globalPosition().y());
		qDebug() << str2;
		// 返回true表示用户已经拦截处理，不再传递到鼠标按下事件的处理函数
		return true;
	}
	// 其它事件交给父类处理（即默认处理）
	return	QLabel::event(e);
}
```

该函数也是虚函数，可查看手册以拦截所有事件，但是一般不建议拦截。

## 11. 事件过滤器

​	在程序将时间分到时间分发器前进行的一层过滤，与事件分发器类似，但是需要指定控件。

```cpp
bool MainWindow::eventFilter(QObject* watched, QEvent* event) {
    if (watched = ui.label_2) {
        if (event->type() == QEvent::MouseButtonPress) {
            // 强制类型转换,否则无法调用QMouseEvent的方法
            QMouseEvent* event2 = static_cast<QMouseEvent*>(event);
            QString str2 = QString("FilterMouse x = %1    Mouse y = %1").arg(event2->globalPosition().x()).arg(event2->globalPosition().y());
            qDebug() << str2;
            // 返回true表示用户已经拦截处理，不再传递到鼠标按下事件的处理函数
            return true;
        }
    }
    // 其它事件交给父类处理（即默认处理）
	return	QWidget::eventFilter(watched, event);
}
```

​	事件过滤器可以拦截事件分发到子组件的事件分发器上，需要指定组件与事件类型。返回true表示用户已拦截处理，返回false表示不拦截，未处理的事件可以调用父类的事件过滤器来进行默认处理。

## 12. 绘图事件

Qt默认设置了绘图事件，默认为空实现，如果需要设置绘图可以重载该实现。

```c++
// 类内重载
void paint::paintEvent(QPaintEvent* event) {
    // 指定绘图到的组件
	QPainter painter(this);
    // 可以设置笔，笔可以修改颜色、粗细等
	QPen pen(Qt::red, 3);
	QPen pen2(Qt::yellow, 3);
    // 可以设置刷子，刷子可以填充图形
	QBrush brush(Qt::green);
	pen.setStyle(Qt::DashLine);
    // 绘图使用painter，可以绘制各类型图案，设置Pen后可以使用设置的颜色
	painter.setPen(pen);
	
	painter.drawLine(0, 0, 100, 100);
	painter.drawEllipse(100, 100, 100, 100);
	painter.setPen(pen2);
    // 设置刷子后可使用指定的内容填充
	painter.setBrush(brush);

	painter.drawRect(200, 200, 100, 100);
	painter.drawText(QRect(200,200,100,100),"Hello World");
}
```

绘图的Pen\Painter\Brush有许多其它的属性可以设置，可以在手册中查看。

QPainter还可以进行一些例如抗锯齿、移动画笔位置等操作：

```c++
	QPainter painter(this);
	painter.drawEllipse(QPoint(100, 100), 100, 100);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.drawEllipse(QPoint(300, 100), 100, 100);
	painter.translate(400, 100);
	painter.save();
	painter.drawRect(QRect(0, 0, 100, 100));
	painter.translate(100, 100);
	painter.restore();
	painter.drawRect(QRect(0, 0, 100, 100));
```

与其它组件结合时，可以调整图的位置重新设置图的属性。

```c++
// 使用connect连接组件的信号，并在槽函数中设置修改一些绘图的属性，使用update更新
connect(ui.horizontalSlider, &QSlider::valueChanged, [=]() {
		this->place = this->getPlace(ui.horizontalSlider->value());
    	// 更新绘图
		update();
		});
```

其它绘图设备类型：

QPixmap 专门为图像在屏幕上的显示做了优化

QBitmap是QPixmap的子类，黑白图片

QImage专门为图片的像素级访问做了优化

QPicture可以记录和重现QPainter的各类命令

```c++
	QPixmap pixmap(300, 300);
	pixmap.fill(Qt::white);
	QPainter painter(&pixmap);
	painter.drawEllipse(150, 150, 100, 100);
	pixmap.save("F:\learn\QT\QtWidgetsApplication3\source\pix.png");
```

无论是什么设备（QT组件、QPixmap等），都需要用一个设置为该设备的QPainter进行绘图。

使用Picture时，可以将之间的绘图操作记录为一个任意后缀名的文件，之后加载该文件可以重新自动复现上次报错的绘图记录。

```c++
	// 绘图并保存
	QPicture picture;
	QPainter painter(&picture);
	painter.setPen(Qt::red);
	painter.setBrush(Qt::blue);
	painter.drawRect(0, 0, 100, 100);
	painter.end();
	picture.save("F:\learn\QT\QtWidgetsApplication3\source\pic.zt");
```

```c++
// 加载并绘制
	QPainter painter(this);

	painter.drawPixmap(this->place, 0, QBitmap(":/source/image-20241223135839705.png"));
	QPicture picture;
	picture.load("F:\learn\QT\QtWidgetsApplication3\source\pic.zt");
	painter.drawPicture(0, 0, picture);
```

## 13. 文件交互

使用QFile进行文件的读写操作，通过QIODevice设置打开文件的方式，可以选择一次性全部读取，也可以按照行读取。

默认编码格式为UTF-8，可以使用QTextCode修改格式。

```c++
	connect(ui.pushButton, &QPushButton::clicked, [=]() {
		QString filename = QFileDialog::getOpenFileName(this, "Open File", "F:\\");
		ui.lineEdit->setText(filename);
		//QTextCodec* codec = QTextCodec::codecForName("UTF-8");
		QFile file(filename);
		file.open(QIODevice::ReadOnly);
		// 一次性读取文件内容
		//QByteArray array =  file.readAll();
		//ui.textEdit->setText(array);
		// 按行读取文件内容
	/*	QByteArray array;
		while (!file.atEnd()) {
			array += file.readLine() + "\n";
		}*/
		// 将读取到的内容设置到文本编辑器中
		ui.textEdit->setText(file.readAll());
		// 关闭文件
		file.close();
		// 进行写文件
		file.open(QIODevice::Append);
		// 追加写入
		file.write("AIRPORAL");
		file.close();
		}
```

获取文件信息：QFileInfo

```c++
QFileInfo info(filename);
qDebug() << info.fileName() << info.size() << info.suffix();
qDebug() << info.birthTime().toString("yyyy/MM/dd hh:mm:ss");

```

# 三：QT项目

































