## § 主机与虚拟机双向互传
### 安装VMware Tools
1. 开启虚拟机后，在vmware菜单栏选择VM -> Install VMware Tools。

1. 在虚拟机内会出现相应的安装文件(Ubuntu虚拟机的桌面上、Win虚拟机的此电脑中)。

1. 在虚拟机内运行并安装Tools。

1. 安装完成后可以实现双向的复制粘贴。但是从Win(主)向Ubuntu复制文件夹可能有问题。

### 安装Xftp
Xftp是一个功能强大的SFTP、FTP 文件传输软件。
使用了 Xftp 以后，Win 用户能安全地在 UNIX/Linux 和 Windows PC 之间传输文件。

1. 进入[官方免费版下载网址](
https://www.netsarang.com/zh/free-for-home-school/)，
填写注册信息，完成后会将下载链接发送到注册邮箱。

1. 安装并打开Xftp。在虚拟机命令行Ping主机IP，确保网络正常。

1. 检查Ubuntu的SSH服务是否开启。输入命令`ps -e|grep ssh`，
输出“ssh-agent”和“sshd”，否则服务未启动或未安装SSH。
通过`sudo apt install openssh-server`安装SSH。
通过`/etc/init.d/ssh start`启动SSH服务。

1. 在虚拟机的Xftp中新建会话，需要用到主机IP、用户名、密码。

