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
The two necessary dependencies are cairo and ffmpeg. LaTeX is strongly 
recommended, as it is necessary to have access to the Tex and MathTex classes.
```bash
sudo apt update && apt upgrade -y
# Install cairo
sudo apt install libcairo2-dev
# Install ffmpeg
sudo apt install ffmpeg
# Install pango
sudo apt install libpango1.0-dev
```

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
├─ main.py                   % 主程序(不要与manim库文件重命)
└─ ...
```

在素材文件夹`assets/`为 manim 的环境路径，可以使用文件名(后缀可省略)定位到对应
的文件。

### Template
```python
import os
import time
from manim import *

class DemoOfManim(Scene):
    def construct(self):
        # create a object
        obj = Square()
        # change propertity
        obj.set_fill(BLUE, opacity=1.0)
        # add objects to scene
        self.add(obj)
        # play animations
        self.play(Rotate(obj,PI/2), run_time=2)
        #  self.play(obj.animate(run_time=2).flip())
        #  self.play(obj.animate(run_time=2).scale(3,about_point=np.array([1,1,0])))
        #  self.play(ApplyMethod(obj.scale, {'scale_factor':3}, run_time=2))
        #  self.play(ApplyMethod(obj.flip, 1*UP+2*RIGHT), run_time=2)
        self.wait(5)

if __name__ == '__main__':
    os.system('manim -pql ' + __file__ + ' DemoOfManim')
    time.sleep(10)
```


## ✨ MObjects
> 使用方法请参考源码。

### Transport methods
MObject 的简单动画都是通过补间完成，如 `MObject.rotate` 等并不是逐帧旋转，而只
是始末状态的相对位置发生了旋转。

```bash
# Displacement
shift, scale, rotate_about_origin, rotate, flip, stretch, wag,
# Functions
apply_function, apply_function_to_position, apply_function_to_submobject_positions,
apply_matrix, apply_complex_function, 
##
repeat, reverse_points, apply_points_function, apply_points_function_about_point, 
pose_at_angle, 
# Positioning methods 
center, align_on_border, to_corner, to_edge, next_to, shift_onto_screen, 
is_off_screen, stretch_about_point, rescale_to_fit, scale_to_fit_width, 
stretch_to_fit_width, scale_to_fit_height, stretch_to_fit_height, scale_to_fit_depth,
stretch_to_fit_depth, set_coord, set_x, set_y, set_z, space_out_submobjects, 
move_to, replace, surround, put_start_and_end_on, add_background_rectangle, 
add_background_rectangle_to_submobjects, 
add_background_rectangle_to_family_members_with_points,
# Color functions
set_color, set_color_by_gradient, set_colors_by_radial_gradient, 
set_submobject_colors_by_gradient, set_submobject_colors_by_radial_gradient,
to_original_color, fade_to, fade, get_color, 
##
save_state, restore, reduce_across_dimension, nonempty_submobjects, get_merged_array, 
get_all_points, 
# Getters
get_points_defining_boundary, get_num_points, get_extremum_along_dim, 
get_critical_point, get_edge_center, get_corner, get_center, get_center_of_mass, 
get_boundary_point, get_midpoint, get_top, get_bottom, get_right, get_left, 
get_zenith, get_nadir, length_over_dim, get_coord, get_x, get_y, get_z, get_start, 
get_end, get_start_and_end, point_from_proportion, get_pieces, 
get_z_index_reference_point, has_points, has_no_points, 
# Match other mobject properties
match_color, match_dim_size, match_width, match_height, match_depth, match_coord, 
match_x, match_y, match_z, align_to
# Family matters
split, get_family, family_members_with_points, arrange, arrange_in_grid, sort, 
shuffle, invert
```


### VGroup
> 函数在调用多个参数时，在列表、元组、集合、字典及其他可迭代对象作为实参，并在
前面加* ，如*(1,2,3) 解释器将自动进行解包然后传递给多个单变量参数（参数个数要
对应相等），对于可变参数的函数则会将解包后的变量全部以实参传入。详情见
[python 中 * 的用法](https://www.cnblogs.com/yinyoupoet/p/13287344.html)

`VGroup.arrange_submobject()` returns `MObject`。

## ✨ Animation
```python
class Demo(Scene):
    def construct(self):
      # play animation
      self.play(Animation)
```

### Common animation
Rotate, AnimationGroup, 

### Updater
```python
MObject.add_updater(func)
```
If `dt` is a parameter of `func`, this updater will keep update while `self.wait()`.
Otherwise, the animation will pause until next `self.play()`.

**Methods:** 
```bash
add_updater, remove_updater, clean_updater
```

### DecimalNumber
`DecimalNumber` is a value monitor.

### ValueTracker
`ValueTracker` is a unvisible value recorder which usually bounded with `Updater`
or `DecimalNumber`. It can control updater, and be displayed by `DecimalNumber`.

```python
class DemoOfUpdater(Scene):
    def construct(self):
        t = ValueTracker(-5)
        sq = Dot().add_updater(lambda x: x.set_x(t.get_value()))
        text = Tex("This a dot.").add_updater(lambda x: x.next_to(sq))
        decimal = DecimalNumber(
            num_decimal_places=3,
            include_sign=True,
        ).add_updater(lambda x: x.set_value(t.get_value()))

        self.add(sq, text, decimal)
        self.play(t.animate.set_value(4))
```


## ✨ Camera
```python
class DemoOfManim(GraphScene, MovingCameraScene):
    def construct(self):
        text=MathTex("\\nabla\\textbf{u}").scale(3)
        square=Square()

        self.add(text,square)
        # Save the state of camera
        self.camera.frame.save_state()
        # Animation of the camera
        self.play(
            self.camera.frame.animate.set_width(text.get_width()*1.2).move_to(text)
        )
        self.wait()
        self.play(Restore(self.camera.frame))
        self.wait()
```


## ✨ Configuration
> [References](
https://docs.manim.community/en/stable/tutorials/configuration.html)

```python
# Configuration example
config.background_color = WHITE
```


## ✨ Mobject
### Geometry
常用的几何形状如下: 

```bash
"TipableVMobject", "Arc", "ArcBetweenPoints", "CurvedArrow", "CurvedDoubleArrow", 
"Circle", "Dot", "AnnotationDot", "LabeledDot", "Ellipse", "AnnularSector", 
"Sector", "Annulus", "Line", "DashedLine", "TangentLine", "Elbow", "Arrow", 
"Vector", "DoubleArrow", "CubicBezier", "Polygram", "Polygon", "RegularPolygram", 
"RegularPolygon", "Star", "ArcPolygon", "ArcPolygonFromArcs", "Triangle", 
"ArrowTip", "Rectangle", "Square", "RoundedRectangle", "Cutout", "Angle", 
"RightAngle", "ArrowCircleFilledTip", "ArrowCircleTip", "ArrowSquareTip", 
"ArrowSquareFilledTip"
```


### Other Mobjects
**SVGMobject** 
```python
obj = SVGMobject(fname)
```
**ImageMobject** 
```bash
obj = ImageMobject(fname)
```
**TextMobject** 
```bash
obj = TextMobject("contents")
```
可使用的文字对象如下: 
| Mobject             | Introduction                                            |
|---------------------|---------------------------------------------------------|
| BulletedList        | Examples                                                |
| MathTex             | A string compiled with LaTeX in math mode               |
| SingleStringMathTex | Elementary building block for rendering text with LaTeX |
| Tex                 | A string compiled with LaTeX in normal mode             |
| TexSymbol           | Purely a renaming of SVGPathMobject                     |
| Title               | Examples                                                |

使用中文字体的方法为
```bash
tex = Tex('中文', tex_template=TexTemplateLibrary.ctex)
```


## ✨ Scene

## ✨ Utilities and other modules
### ParametricFunction

