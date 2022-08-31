✨ 现场总线与数据传输的基本概念
-------------------------------
### 现场总线
现场总线的一般形式如下:  
`传感器 --- 变送器 --- 收发器` --- `总线` --- `收发器 --- 上位机`  
`设备 --- 收发器` --- `总线` --- `收发器 --- 上位机`

1. **变送器** : 也称(后端)放大器。将传感器的模拟输出放大，一般输出模拟信号。
   与收发器集成在一起时，可以输出数字信号。部分传感器不需要放大器。
1. **收发器** : 包括传感器的采集卡。调制和解调，实现 AD/DA 转换，用于设备在总
   线上交换数据。不同的收发器可能需要对应厂商提供的 SDK 。

### 电气接口
现场总线中常见的电气接口有:
1. DB9(sub-d9)
1. RJ45
1. USB

设备如何接入总线取决于物理层协议。如 RS485 和 CAN 只需要两根线， RS232 需要三根
线， TCP/IP 使用 RJ45 接口。

### 传输协议
1. 硬件协议: 物理层协议。规定物理层的电平高低，解决 0 和 1 的可靠传输，如
   RS485 、 RS232 等。
1. 软件协议: 应用层协议。解决传输目的，通常有 Modbus 、 TCP/IP 、 CANopen 等。

### 报文格式
衡量数据大小的单位有: 
1. 位(bit/b): 电脑记忆体中的最小单位，二进制计算机中每个 bit 为一个二进制位，
   可以表示 0 或 1 。
1. 字节(Byte/B): 记忆体存储资料的基本单位，数据传输单位为字节。
1. 字(word): 特定语言的存储需要两个字节，如中文。通常是一个寄存器的大小。

其数量关系为 `1 word = 2 byte`; `1 byte = 8 bit`;  
与传输速率相关的 b 指 bit ，与容量有关的 b 指 Byte 。



✨ 常见的传输协议
-----------------
### Modbus
> Modbus 是一类 **应用层** 报文传输协议的统称，定义了控制器能够识别和使用的消
> 息结构。

Modbus 的主要特点有: 
1. **一主多从** 。只有主机可以发送请求，且同一时间只能向一个从机发送请求。从机
   负责处理和存储数据，接受主机的请求进行响应。
1. Modbus 总线只是传输数据，没有仲裁机制。

Modbus 的报文结构一般有以下几类:  
起始位、从机地址、功能码、数据位、校验位、结束位。

Modbus 中从机的存储区:  
Modbus 协议规定了从机数据模型的 4 个存储区，每个存储区划定了地址范围，与实际设
备的物理存储地址对应。  
主机向从机获取数据时，需要发送起始地址和字节数。  
主机读写从机的存储区，实际就是读写从机设备对应的实际存储空间。
| 区号 | 名称       | 数据类型       | 地址范围      |
|------|------------|----------------|---------------|
| 0 区  | 输出线圈   | 可读可写布尔量 | 00000 ~ 09999 |
| 1 区  | 输入线圈   | 只读布尔量     | 10000 ~ 19999 |
| 3 区  | 输入寄存器 | 只读寄存器     | 30000 ~ 39999 |
| 4 区  | 保持寄存器 | 可读可写寄存器 | 40000 ~ 49999 |

Modbus 协议目前主要用于串口、以太网。串口通信的变种有 Modbus-RTU 和
Modbus-ASCII ；以太网连接的变种有 Modbus-TCP 。
1. **Modbus-RTU** : 十六进制表示数据。如 10 为 0x0A ，在总线上的形式为 `0000
   1010`。
1. **Modbus-ASCII** : ASCII 码表示数据。如 10 为 "0" 和 "1"，在总线上的形式为
   `0011 0001 0011 0000`。
1. **Modbus-TCP** : 利用以太网 TCP/IP 实现连接，可以理解为发生在 TCP 上的应用
   层协议。物理层、数据链路层、网络层和传输层都是基于 TCP 协议。
1. **ModbusPlus** : 一种典型的令牌总线网。

**参考文献** :
1. [Modbus 协议简介与 FreeMODBUS 移植](
https://blog.csdn.net/xiaoluoshan/article/details/53574988)
1. [详解 Modbus 通信协议, STM32 嵌入式开发](
https://mp.weixin.qq.com/s?__biz=MzI1MDg4OTMwMw==&mid=2247503331&idx=5&sn=29c44a8b4f128b92e83a2a6d2ae005aa)

-----------------------------------------------------------------------------


### CAN
> CAN 规定了物理层和数据链路层。

CAN 通信的主要特点有:
1. 需要用户自定义应用层，所以不同的厂商需要用不同的收发器及配套的 SDK 。
1. 传输速度最高到 1Mbps ，通信距离最远到 10km 。
1. 无损位仲裁机制，去中心化，分布式原则，多主结构。
1. 不同的 CAN 标准仅物理层不同。
1. 事件驱动的协议，11989-4 标准引入了时间触发机制。

#### 物理层
CAN 有三种接口器件: M12 小型连接器、 Open5 连接端子、 DP9 插座。

CAN 收发器通过 CAN_H 、 CAN_L 以并联方式接入总线。

对于高速 CAN 通信，在由一组电缆直接连接的 CAN 网络内，需要在直线拓扑结构距离最
远的两个端点上各加一个 120 Ω 的终端电阻，以匹配电缆的特征阻抗，防止信号反射。

CAN 收发器的输出信号等效于从上拉电阻输出，所以低电平会覆盖高电平。
多个节点连接，只要有一个为低电平，总线就为低电平(显性)，只有所有节点输出高电平
时，才为高电平(隐形)，所谓"线与"。

#### 数据链路层
CAN的通信帧分成五种，分别为数据帧、远程帧、错误帧、过载帧和帧间隔。
* 远程帧: 也称遥控帧。某个节点向另一个节点请求其发送特定数据时所用的帧。

仲裁段: 显性优先，帧ID越小优先级越高。CAN 控制器在发送数据的同时监控总线电平，
如果电平不同则停止发送并做其他处理。如果该位位于仲裁段，则退出总线竞争；如果位
于其他段，则产生错误事件。


**参考文献** :
1. [详解 CAN 总线, STM32 嵌入式开发](
https://mp.weixin.qq.com/s?__biz=MzI1MDg4OTMwMw==&mid=2247504469&idx=1&sn=5a4c3f7d0d4c01134ee2b02e2425151b)

-----------------------------------------------------------------------------


### TCP/IP

