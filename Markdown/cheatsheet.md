## § Table of Contents
1. [超实用的语法](##-§-超实用的语法)
    + [备注](###-备注)
    + [脚注/参考文献](###-脚注/参考文献)
    + [目录](###-目录)
1. [Reference](##-§-Reference)

> 本文提及的方法或技巧可能需要特定的编辑器。
Github网页可能无法正确显示，如需在Github实现类似效果需要安装插件，
或者用基本的Markdown语法和适当的渲染。

## § 超实用的语法
### 备注
可以用如下方法生成Markdown备注[^comment]，注意第二行要 **空四格** 。
```markdown
[^_^]:
    comment
```

### 脚注/参考文献
添加脚注的方法为`[^foo]`，在任意位置以`[^foo]: bar`的语法格式添加脚注的注释。
脚注会按正文中的引用顺序排列在文档最末尾，并且可以双向跳转，通常用来引用参考文献。

### 目录
在需要生成目录的地方输入`[toc]`即可自动生成目录，
但是这样会生成所有层级的目录。
手动生成目录的方式是以链接的方式将需要列举的标题罗列出来[^toc]。
手动生成目录时，在链接部分需要将空格` `替换为短横线`-`。


## § Reference
[^toc]: [GitHub Wiki TOC generator](
http://ecotrust-canada.github.io/markdown-toc/)
[^comment]: [Commonts in Markdown](
https://stackoverflow.com/questions/4823468/comments-in-markdown)

