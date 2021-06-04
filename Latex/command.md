1. [do](#✨-command-%5Cdo)

## ✨ Command `\do`
> 简而言之，\do 就是一个命令的别名。常在需要重复执行某命令时使用。

### What does \do mean
`\do` is short for `\performsomeactiononanargument` and is typically redefined 
depending on the `action` that needs to be performed.

For example, consider the `verbatim` environment (or `\verb`). We know that the 
`verbatim` environment allows you to use certain characters that are otherwise 
restrictive in their use or application, like `$`, `\`, `~`, `&`, `%`, ... So, 
in order to set up the environment to treat these characters as *characters* 
and not their aforementioned special behaviour, \@verbatim does (see
[latex.ltx](
https://www.tug.org/svn/texlive/trunk/Master/texmf-dist/tex/latex/base/latex.ltx?view=co):

```tex
\let\do\@makeother \dospecials
```
Where
```tex
\def\@makeother#1{\catcode`#112\relax}
\def\dospecials{\do\ \do\\\do\{\do\}\do\$\do\&%
  \do\#\do\^\do\_\do\%\do\~}
```

### Some application
[etoolbox](https://ctan.org/pkg/etoolbox) adopted this usage/notation for 
"performing some action on an argument" when processing a list of items. For 
example, when calling

```tex
\docsvlist{first,second,third,last}
```

`\do` is applied to each of the elements sequentially, as in `\do{first}`, 
`\do{second}`, `\do{third}` and `\do{last}`. This allows the user to (re)define
what the meaning of `\do` is exactly. Here's an example:

```tex
\documentclass{article}

\usepackage{etoolbox}

\begin{document}

\begin{enumerate}
  \renewcommand{\do}{\item}
  \docsvlist{first,second,third,last}
\end{enumerate}

\noindent
\begingroup
\renewcommand{\do}[1]{\textbullet~#1 \quad}
\docsvlist{first,second,third,last}
\endgroup

\end{document}
```

In a very limited context, `\do` is sometimes used to delimit macro arguments. 
But in that context it makes literal sense, even though a completely different 
delimiter could have been used. One example of this is used in `\@whilenum` or 
as part of `\@for` (loop) constructions.


### Refernece
1. [What does \do do? --- StackExchange](
https://tex.stackexchange.com/questions/331483/what-does-do-do)

