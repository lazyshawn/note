[toc]

### 添加中文修改后无法保存
之前修改的文件存在一部分中文，但上一个负责人提交时，未按规定使用utf-8编码，
导致中文处全为乱码。

用VIM删除乱码并重新输入原有的中文，最后保存时出现no write since last change，
无法保存。强行使用“！”保存后，再次打开文件出现一堆颠倒的“？”。

解决方法是保存时指定编码方式:`:w ++enc=utf-8`

**ref:** [VIM保存文件，出现no write since last change解决](
https://blog.csdn.net/zrq293/article/details/106619113/)

