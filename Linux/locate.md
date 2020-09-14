## What does command "locate" do
在Ubuntu后台索引的数据库中搜索给定的文件。
在Ubuntu中，这个数据库文件位于`/var/cache/locate/locatedb`。
一般来说，该文件每天通过cron自动更新，也可以执行`sudo updatedb`手动更新。
