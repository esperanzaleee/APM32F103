在mutex_sample.c中

![](./figure/q1.jpg)

线程2的优先级比线程1高，应该先运行线程2

在线程2中是先打印再加一的  那这样的话打印的第一个数应该是0。

但此时，串口先打印数字1。

![](./figure/q2.jpg)

此时线程中优先级情况如图

![](./figure/q3.jpg)

若将线程1的优先级改为20，则打印第一个数为0；当线程优先级低于20，打印的第一个数都为1。

很迷惑。。。。。。