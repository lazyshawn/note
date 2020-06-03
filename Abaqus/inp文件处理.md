1. Abaqus/CAE在提交作业后，在工作路径下生成相应的.inp文件。
工作路径的查看和设置方法：File--Set Work Directory。

2. .inp文件批量处理方法：
在.txt文件写入下面命令，保存后修改成.bat文件，然后运行即可批量处理。
call abaqus job=jobname1 int
call abaqus job=jobname2 int
其中，jobname(i)为.inp文件的文件名。

3. CAE中导入.inp文件方法：
File--Import--Model

