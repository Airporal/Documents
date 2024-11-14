# Python知识

------

[TOC]

------



## 1.常用函数汇总

| 函数                               | 作用                                                         |
| ---------------------------------- | ------------------------------------------------------------ |
| zip()                              |                                                              |
| isinstance(n,(int,str,list))       | 判断n是否是后面的类型之一                                    |
| and or not                         | 逻辑与或非                                                   |
| as                                 | 异常捕获、文件、包导入时命名                                 |
| assert                             | 判断条件为假则报错AssertionError                             |
| break continue                     | 跳出循环 继续执行下一次循环                                  |
| del                                | 删除序列或变量                                               |
| complie(str,"<string>",''exec“)    | 将文本转为执行代码的标准文本块                               |
| exec(str)                          | 将文本以python语句执行                                       |
| yield                              | 和return一样用作函数返回，但函数成为可迭代对象，每次调用从上次执行点继续执行。 |
| next                               | 调用一次可迭代对象                                           |
| lambda                             | 定义匿名函数                                                 |
| raise                              | 抛出异常                                                     |
|                                    |                                                              |
|                                    |                                                              |
|                                    |                                                              |
| format()                           | 字符串的方法，用于格式化字符串，配合{}                       |
| dict()                             | 用于创建一个字典                                             |
| dir()                              | 返回对象的所有属性、方法                                     |
| hex()                              | 将十进制转化为十六进制的字符串表示                           |
| divmod(m,n)                        | 返回m除以n的商和余数                                         |
| id()                               | 获取对象的储存地址                                           |
| sorted(cmp,key,reverse)            | 排序。reverse决定升序或降序，key决定用来比较的是各自的哪一个值，cmp为进行一个函数后将结果比较。 |
| oct()                              | 转化为8进制                                                  |
| bin()                              | 转化为2进制                                                  |
| sum()                              | 对序列进行求和                                               |
| filter()                           | 对序列进行过滤                                               |
| zip(m,n)                           | 将m,n的相同序号的元素各自组合，得到一个元组，返回迭代器      |
| zip(k)                             | 可迭代对象k按照行进行输出，zip(*k)按列输出，返回迭代器       |
| map(fun,iter)                      | 将可迭代对象iter的所有元素按照fun做映射，返回迭代器。        |
| reversed()                         | 返回一个反转的迭代器                                         |
| round()                            | 返回浮点数的四舍五入值                                       |
| **kwargs                           | 将函数的参数和值储存在字典kwargs中                           |
| *args                              | 将函数的参数储存在元组args中                                 |
| _ _len\_ _                         | 定义对类的实例使用len时的结果                                |
| _ _add\_ _（self,m,n...)           | 定义对类的实例使用+时的结果                                  |
| _ _getitem\_ _（self,key)          | 定义对类的实例可以使用self[key]获取值                        |
| _ _eq\_ _（self,m,n...)            | 定义对类的实例使用==时的结果                                 |
| _ _setitem\_ _（self,key)          | 定义对类的实例可以使用self[key]=value赋值                    |
| @property                          | 将类的函数方法转化为属性，调用时不需要加（）                 |
| value._ _hash\_ _（)               | 通过哈希算法映射变量的值                                     |
|                                    |                                                              |
| open(file,mode,buffering,encoding) | 以model：只读‘r'、写入'w'、追加’a'，打开file，并设置缓存buffering和编码格式encoding |
| f.readline()                       | 将文件的所有数据放在列表中                                   |
| f.close                            | 关闭文件                                                     |
| PyMySQL库                          | 提供对mysql数据库操作的函数                                  |
| re库                               | 提供正则表达式查找字符的函数                                 |
| smtplib和email库                   | 提供制作、发送邮件的函数                                     |
| Flask库、Django库                  | 基于jinja2的两种python Web应用开发框架，包含网页处理、数据库管理等功能 |
| pygame库                           | python游戏开发库                                             |
| itchat库                           | python的微信接口库                                           |
| scipy库                            | 提供插值、优化、统计等科学计算库                             |
| seaborn库                          | 基于matplotlib库的高级绘图库                                 |
| math模块                           | 提供浮点数学运算的函数                                       |
| cmath模块                          | 提供复数运算的函数，复数用n+mj表示                           |
| random模块                         | 生成不同分布的各种随机数                                     |
| feedparser模块                     | 提供解析Web上RSS格式的xml的包                                |
| sklearn模块                        | python中对机器学习数据处理、分析等的封装                     |
| snownlp模块                        | python的中文自然语言处理库                                   |
|                                    |                                                              |



## 2.网络爬虫



## 3.数据可视化

> 使用numpy、scipy对数据进行计算,使用matplotlib、seaborn对数据进行可视化处理。

## 4.pytorch

