## 图表特殊字符标注
1. 添加**latex** 语法, 如使用下列命令即可在编号框中加入latex数学公式编辑的字符:
```
x = -pi:pi/20:pi;
plot(x,cos(x),'-ro',x,sin(x),'-.b')
h = legend('$\cos{\hat a}$','$\sin{x}$',2);
set(h,'Interpreter','latex','fontsize',20) 
```

