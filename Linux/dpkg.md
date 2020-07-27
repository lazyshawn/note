## dpkg: warning: files list file for package 'x' missing
[参考教程](https://serverfault.com/questions/430682/dpkg-warning-files-list-file-for-package-x-missing)

**可能原因**：info文件夹中软件包损坏，需要重新安装。

**解决方案**：重新安装运行出错的命令提示的软件包。
```shell
for package in $(apt-get upgrade 2>&1 |\
                 grep "warning: files list file for package '" |\
                 grep -Po "[^'\n ]+'" | grep -Po "[^']+"); do
    apt-get install --reinstall "$package";
done
```
其中`apt-get upgrade`改成报错的命令。

## dpkg: error processing package
```shell
sudo mv /var/lib/dpkg/info/ /var/lib/dpkg/info_old/
sudo mkdir /var/lib/dpkg/info/
sudo apt-get update
sudo apt-get -f install
sudo mv /var/lib/dpkg/info/* /var/lib/dpkg/info_old/
sudo rm -rf /var/lib/dpkg/info
sudo mv /var/lib/dpkg/info_old/ /var/lib/dpkg/info/
sudo apt-get update
```

