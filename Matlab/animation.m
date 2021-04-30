% Animation produce and save
clear; clc;

% 创建avi文件对象
aviobj = VideoWriter('test.avi','Uncompressed AVI');
open(aviobj)

% 动画部分代码
fig=figure;
t = linspace(0,2.5*pi,40);
fact = 10*sin(t);
[x,y,z] = peaks;
for k=1:length(fact)
    h = surf(x,y,fact(k)*z);
    axis([-3 3 -3 3 -80 80])
    axis off
    caxis([-90 90])
    % 获取当前画面
    % 如果在这里把F记录下来，则可以用movie函数播放
    F = getframe(fig);
    %加入avi对象中
    writeVideo(aviobj,F);
      
    %转成gif图片,只能用256色
    im = frame2im(F);
    [I,map] = rgb2ind(im,256);
    %写入 GIF89a 格式文件   
    if k == 1;
        imwrite(I,map,'test.gif','GIF', 'Loopcount',inf,'DelayTime',0.1);
    else
        imwrite(I,map,'test.gif','GIF','WriteMode','append','DelayTime',0.1);
    end   
end

% 关闭gif
close(fig);
% 关闭avi对象
close(aviobj);

% Reference
% https://blog.csdn.net/song430/article/details/104006620

