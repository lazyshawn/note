1. [✨ Python for Data Science](#✨-python-for-data-science)
1. [✨ Trouble shooting](#✨-trouble-shooting)
    + [Offset” and “scientific notation”](#“offset”-and-“scientific-notation”)
1. [✨ Reference](#✨-reference)

## ✨ Python for Data Science
Tutorial[^1]. Cheat sheet[^2]. Demo and Gallery[^3].

Other problem not mentioned above will be listed at below.

## ✨ Trouble shooting
### "Offset" and "scientific notation"
> [prevent scientific notation in matplotlib.pyplot --- StackOverflow](
https://stackoverflow.com/questions/28371674/prevent-scientific-notation-in-matplotlib-pyplot)

In matplotlib axis formatting, "scientific notation" refers to a multiplier for 
the numbers show, while the "offset" is a separate term that is added.

Use `ax.ticklabel_format` to prevent `offset`, `scientific notation` or both of 
them. One can also set this for specific axis by adding ``axis= '<axis>'`` 
argument. Where `<axis>` could be `x`, `y`, `z`, or `both`.
```python
ax.ticklabel_format(useOffset=False, style='plain')
```


## ✨ Reference
[^1]: [Matplotlib Tutorial: Python Plotting](https://www.datacamp.com/community/tutorials/matplotlib-tutorial-python)
[^2]: [Matplotlib Cheat Sheet: Plotting in Python](https://www.datacamp.com/community/blog/python-matplotlib-cheat-sheet)
[^3]: [Gallery-matplotlib documentation](https://matplotlib.org/stable/gallery/index.html)


