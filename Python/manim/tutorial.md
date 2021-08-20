1. [✨ What is manim](#✨-what-is-manim)
   - [Introduction](#introduction)
   - [Some helpful guide](#some-helpful-guide)
1. [✨ Project](#✨-project)
   - [Install](#install)
   - [Structure](#structure)
   - [Template](#template)
1. [✨ Objects](#✨-objects)

## ✨ What is manim
### Introduction
Manim is an engine for precise programmatic animations, designed for creating 
explanatory math videos.

Note, there are two versions of manim.
[3b1b/manim](https://github.com/3b1b/manim) began as a personal project by the 
author of 3Blue1Brown for the purpose of animating those videos. In 2020 a 
group of developers forked it into what is now the community edition, 
[ManimCommunity/manim](https://github.com/ManimCommunity/manim) , with a goal 
of being more stable, better tested, quicker to respond to community 
contributions, and all around friendlier to get started with. This tutorial 
falls into the latter.

### Some helpful guide
1. [manim.community](https://www.manim.community/)
1. [bilibili](https://search.bilibili.com/all?keyword=manim教程)
1. [3blue1brown](https://www.3blue1brown.com/)

## ✨ Project
### Install
Manim Community runs on Python 3.7+. If you’d like to just use the library, you 
can install it from PyPI via pip:
```bash
pip3 install manim
```

### Structure
通常情况下，manim 项目的文件结构如下所示。
```bash
manim
├─ assets                     % 素材
│  ├─ raster_images           % 图片
│  │  └─ picture.png
│  ├─ sounds                  % 声音文件(一般不用)
│  └─ svg_images              % 矢量图
│     └─ svg_file.svg
├─ manimlib                   % manim库
│  └─ ...
├─ manim.py                   % 主程序
└─ ...
```

在素材文件夹`assets/`为 manim 的环境路径，可以使用文件名(后缀可省略)定位到对应
的文件。

### Template
```bash
import os
import time
from manim import *

class DemoOfManim(Scene):
    def construct(self):
        obj = Square()  # create a square
        obj.set_fill(BLUE, opacity=1.0)
        self.add(obj)
        self.play(Rotate(obj,PI/2), run_time=2)
        #  self.play(obj.animate(run_time=2).flip())
        #  self.play(obj.animate(run_time=2).scale(3,about_point=np.array([1,1,0])))
        #  self.play(ApplyMethod(obj.scale, {'scale_factor':3, run_time=2)
        #  self.play(ApplyMethod(obj.flip, 1*UP+2*RIGHT), run_time=2)
        self.wait(5)

if __name__ == '__main__':
    os.system('manim -pql ' + __file__ + ' DemoOfManim')
    time.sleep(10)
```


## ✨ Objects

