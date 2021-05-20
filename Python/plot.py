##################################
# Reference
##################################
# 1. [Gallery-matplotlib documentation](https://matplotlib.org/stable/gallery/index.html)
# 1. [Matplotlib Cheat Sheet: Plotting in Python](https://www.datacamp.com/community/blog/python-matplotlib-cheat-sheet)
# 1. [Matplotlib Tutorial: Python Plotting](https://www.datacamp.com/community/tutorials/matplotlib-tutorial-python)

##################################
# 安装库
##################################
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

##################################
# 设置字体
##################################
# 正常显示中文标签
plt.rcParams['font.sans-serif'] = ['SimHei']
# 正常显示负号
plt.rcParams['axes.unicode_minus'] = False


##################################
# 修改配置文件 | Change default rc settings
##################################
#  print(mpl.matplotlib_fname())
#  mpl.rcParams['lines.linewidth'] = 1.5
# X、Y轴标签字体大小
#  mpl.rcParams['xtick.labelsize'] = 10.5
#  mpl.rcParams['ytick.labelsize'] = 10.5
# X、Y轴刻度标签字体大小
#  mpl.rcParams['axes.labelsize'] = 10.5


##################################
# 创建图框
##################################
# fig:
# 1 inch == 2.54 cm, 单栏8.3 cm(3.27 inch), 双栏17.6 cm(6.9 inch)
# figsize=(x inch, y inch), int dpi,
# facecolor=(r,g,b), edgecolor=(r,g,b),
# bool frameon.
cm = 1/2.54
fig = plt.figure(figsize=(17.6*cm, 20*cm))
data = np.linspace(0,10,100)


##################################
# 绘制子图
##################################
# 注意: 每一个子图是独立的，如是否显示图例，需要单独在子图函数下面增加
### 子图1: 二维曲线图
data_1 = np.linspace(0,10,100)

ax1 = fig.add_subplot(321)
# plot: c(color), marker, linewidth
ax1.plot(data_1, np.sin(data_1), 'r--', label='$sin(x)$')
ax1.plot(data_1, np.cos(data_1), 'g-.', label='$cos(x)$')
ax1.plot(data_1, np.sin(data_1)+np.cos(data_1), 'c', label='$sin(x)+cos(x)$')

# legend: 
# loc = upper/lower/center, right/left/center, best(default)
# bbox_to_anchor = (x, y)
ax1.legend(loc='lower right')
ax1.annotate('$sin(0)=0$', xy=(0,0), xytext=(0.2,-1), arrowprops={'arrowstyle': '->'})
ax1.text(0.2, 1.5, r'Demo Equation: $y=\rm{sin}(x)$')
ax1.grid(True)
# set:
# ax.set_foo(bar) == ax.set(foo=bar)
# title, xlabel, xlim, xticks, xticklabels
ax1.set(title  = '正余弦函数图像',
        xlabel = r'时间$\rm{(t/s)}$',
        ylabel = r'位移$\rm{(x/m)}$')
ax1.set_xlim(0, 10)
ax1.set_ylim(-1.5, 1.9)


### 子图2: 饼图
N = 20
np.random.seed(19680801)
theta = np.linspace(0.0, 2 * np.pi, N, endpoint=False)
radii = 10 * np.random.rand(N)
width = np.pi / 4 * np.random.rand(N)
colors = plt.cm.viridis(radii / 10.)

# projection: polar-极坐标绘图
ax2 = fig.add_subplot(322,projection='polar')
ax2.bar(theta, radii, width=width, bottom=0.0, color=colors, alpha=0.5)


### 子图3: 带有误差带的曲线图
ax3 = fig.add_subplot(323)
N = 400
t = np.linspace(0, 2 * np.pi, N)
r = 0.5 + np.cos(t)
x, y = r * np.cos(t), r * np.sin(t)

# Error amplitudes depending on the curve parameter *t*
# (actual values are arbitrary and only for illustrative purposes):
err = 0.05 * np.sin(2 * t) ** 2 + 0.04 + 0.02 * np.cos(9 * t + 2)
# calculate normals via derivatives of splines
tck, u = splprep([x, y], s=0)
dx, dy = splev(u, tck, der=1)
l = np.hypot(dx, dy)
nx = dy / l
ny = -dx / l
# end points of errors
xp = x + nx * err
yp = y + ny * err
xn = x - nx * err
yn = y - ny * err

vertices = np.block([[xp, xn[::-1]], [yp, yn[::-1]]]).T
codes = mpl.path.Path.LINETO * np.ones(len(vertices), dtype=mpl.path.Path.code_type)
codes[0] = codes[len(xp)] = mpl.path.Path.MOVETO
path = mpl.path.Path(vertices, codes)
patch = mpl.patches.PathPatch(path, facecolor='C0', edgecolor='none', alpha=0.3)

ax3.plot(x, y)
ax3.add_patch(patch)
ax3.set(title='误差曲线图')


### 子图4: 直方图
np.random.seed(19680801)

# example data
mu = 100  # mean of distribution
sigma = 15  # standard deviation of distribution
x = mu + sigma * np.random.randn(437)
num_bins = 50

ax4 = fig.add_subplot(324)
# the histogram of the data
n, bins, patches = ax4.hist(x, num_bins, density=True)
# add a 'best fit' line
y = ((1 / (np.sqrt(2 * np.pi) * sigma)) *
     np.exp(-0.5 * (1 / sigma * (bins - mu))**2))

ax4.plot(bins, y, '--')
ax4.set_xlabel('Smarts')
ax4.set_ylabel('Probability density')
ax4.set_title(r'Histogram of IQ: $\mu=100$, $\sigma=15$')


### 子图5: axhspan demo
t = np.arange(-1, 2, .01)
s = np.sin(2 * np.pi * t)

ax5 = fig.add_subplot(313)
ax5.plot(t, s)
# Thick red horizontal line at y=0 that spans the xrange.
ax5.axhline(linewidth=8, color='#d62728')
# Horizontal line at y=1 that spans the xrange.
ax5.axhline(y=1)
# Vertical line at x=1 that spans the yrange.
ax5.axvline(x=1)
# Thick blue vertical line at x=0 that spans the upper quadrant of the yrange.
ax5.axvline(x=0, ymin=0.75, linewidth=8, color='#1f77b4')
# Default hline at y=.5 that spans the middle half of the axes.
ax5.axhline(y=.5, xmin=0.25, xmax=0.75)
# Infinite black line going through (0, 0) to (1, 1).
ax5.axline((0, 0), (1, 1), color='k')
# 50%-gray rectangle spanning the axes' width from y=0.25 to y=0.75.
ax5.axhspan(0.25, 0.75, facecolor='0.5')
# Green rectangle spanning the axes' height from x=1.25 to x=1.55.
ax5.axvspan(1.25, 1.55, facecolor='#2ca02c')


##################################
# 后处理
##################################
# make sure that the plots fit nicely in your figure
plt.tight_layout()
plt.savefig('../foo.svg')
plt.show()


