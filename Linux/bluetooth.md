## How to connect device by bluetooth
### 命令行方式
1. 运行`hciconfig`命令，确认蓝牙模块(适配器)是否被系统识别，避免你插入多个蓝牙设备；
1. 运行`hciconfig hci0 up`，为你的蓝牙设备上电；
1. 运行`bluetoothctl`命令，进入蓝牙设置模式；
1. 运行`scan on`，启动搜索模式；
当找到需要连接的设备时，输入`scan off`，停止探索；
1. 运行`pair <address>`配对蓝牙地址（你需要连接的设备）；
1. 成功配对后Ubuntu会打印配对成功信息；
1. 运行`connect <address>`，完成设备配对。

### 其他命令
1. `hcitool dev`: 查看是否检测到蓝牙适配器;
1. `trust <address>`: 信任设备;

### Reference
1. [Ubuntu下怎么通过命令完成蓝牙的配对和连接
](https://blog.csdn.net/zhuyong006/article/details/89926521)

