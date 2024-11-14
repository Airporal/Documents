# 记录算法题笔记





###### 5.27 递归

```cpp
/**
 * Definition for singly-linked list.
 * 结构体中指针有下一个ListNode的指针：next
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode() : val(0), next(nullptr) {}
 *     ListNode(int x) : val(x), next(nullptr) {}
 *     ListNode(int x, ListNode *next) : val(x), next(next) {}
 * };
 */
class Solution {
public:
    ListNode* reverseList(ListNode* head) {
        // 当链表为【】时，第一个head就是nullptr
        // 不加head!=nullptr会使次情况无解
   
        if (head!=nullptr and head->next != nullptr){
            hh = reverseList(head->next);
            head->next->next = head;
            // 当前链表的下一个要为空，否则无法设置最后的链表指向空
            // 导致出现圈
            head->next = nullptr;
            return hh;
        }
        else{
            return head;
        }
    }
private:
    ListNode* hh;
};
```

CPP中使用递归反转单向链表。

###### 5.28 c++封装的标准字符串、数组类

乘法与位运算的关系：
$$
t<<1 = 2*t
$$
c++提供了string类，避免了char数组实现字符串。基本用法有：

```c++
#include <string>
using namespace std;  // 指定缺省的使名空间。
string str;       // 创建string对象。
str = "data";	//赋值

int len = str.length(); //返回字符串长度
int i = 2;
printf("%d个字符是：%c",i,str[i]); //字符串索引
char c='d';
printf("%s中第一次出现%c的位置是：%d",str,c,str.find(c)); //字符串索引
//在第n个位置插入新的字符
str.insert(n,"new");
```

c++提供了容器类，可用以实现各类型数据的向量，避免使用指针实现数组。

```c++
#include <vector>
#include <algorithm>
using namespace std;
vector<int> name; //实例化一个空的整形数组
vector<int> name(7); //实例化一个包含7个数的整形数组，每个都默认为0
vector<int> name(7,2); //实例化一个包含7个数的整形数组，每个都是2
vector<int> name = {1,2,3,4};// 初始化一个包含1,2,3,4的数组
// name是数组的头指针
//向数组中添加内容
name.push_back(4); //将4加到数组末尾
//索引数组第n个元素
int val = name[n];
name.begin();//数组第一个元素指针
name.end();//数组最后一个元素指针
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
//根据地址删除元素
name.erase(name.begin()+2); // 删除第三个元素
//清空数组
name.clear();
//利用algorithm提供的方法可以找到向量最值下标
auto max_idx = max_element(name.begin(),name.end())
// 返回最大值
auto max = max_element(name.begin(),name.end())

```

###### 5.29 枚举与数学解析

问题可以通过枚举的方法解决，虽然简单，但是带来时间、空间复杂度较高，优化时从数学解析的角度思考问题，得到数学公式以直接求解。

将字符串转化为回文：

```c++
class Solution {
public:
    string makeSmallestPalindrome(string s) {
        int left = 0, right = s.size() - 1;
        while (left < right) {
            if (s[left] != s[right]) {
                s[left] = s[right] = min(s[left], s[right]);
            }
            ++left;
            --right;
        }
        return s;
    }
};

```

###### 6.22 利用vector建立高维度张量

通过vector的镶套可以建立高维度数据：

```cpp
// 建立一个二维数组：
vector<vector<int>> nums;
//建立一个string表：
vector<vector<string>> name;
```

c++字典：

```c++
unordered_map<std::string,std::int> umap={"key1":val1,"key2":val2};
// 根据key1查找值
int va = umap[key1];
// 删除,返回删除的key对应的值，没有就返回0
auto n = umap.erase(key1);
// 查找val对应的key
string a = umap.find(val);
//查找key出现的次数
int num = umap.count(key);
```

###### 6.30 动态规划

