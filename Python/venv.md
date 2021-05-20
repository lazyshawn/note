## ✨ Python virtual environment
Python 由于2.x与3.x的版本中部分库不兼容，
所有对不同的项目往往需要创建对应的[虚拟环境](
https://docs.python.org/zh-cn/3/library/venv.html#venv-def)，
安装各自需要的库。

### Python 3 创建虚拟环境
> [venv -- 创建虚拟环境](https://docs.python.org/zh-cn/3/library/venv.html)
1. 通过执行 `venv` 指令来创建虚拟环境:
```bash
python3 -m venv /path/to/virtual/environment
```

2. 通过执行 `source` 激活虚拟环境:
```bash
source /path/to/venv/bin/activate
```

3. 安装功能库:
```bash
pip3 install matplotlib 
```

4. 安装 Python 3 插件，详情请通过 `help provider-python` 查看:
```bash
python3 -m pip install --upgrade pynvim
```


