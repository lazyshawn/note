## ✨ vim-visual-multi
[vim-visual-multi](https://github.com/mg979/vim-visual-multi) (VM) is a Neovim
plugins.

## ✨ Basic operations
> Note: all '\\' in keybindings means \<Leader\>.
### Create cursors
Press `<C-U/D>` to create multiple cursors, also you enter **cursor mode**.
What can you do with these cursors?

- you can move them: press w, e, b, like vim operations.
- you can issue VM specific commands that will be introduced later.
- you can issue normal mode commands: dw, cw, and etc.
- you can also enter insert mode with i, a, o.

### Extend selections
Press `<S-L/R>` to select a match pattern. Press `n/N` will find next/previous
match and add current match to selections. You can skip current match by `q`, 
and remove cursors and selections with `Q`.

If you press `<Tab>` in cursor mode, you'll switch to **extern mode**. You can
think of it as the VM's `visual` mode which handle multiple visual block.

### Select words
- Press `<C-n>` to select words under the cursor.
- Press `\\c` to cycle the case setting of the current pattern.
- Press `\\w` to converted its pattern whether check for word boundaries.

### Alignment
You can press `\\c` from Visual mode to create a column of cursors at beginning
of each line. There are several ways to align code blocks, using `\\a` command.

- `f<char>\\a` will navigate cursors to the \<char\>, and align them.
- `2\\<=` will align two characters, that are = and ".(`\\>` is alternative)

### Dot
Press the dot (.) can be a very fast way to apply a change at cursors.

- First do some change in Insert mode, and exit Insert mode. Undo the change,
and match selections. Finally Press `.`, which will apply the change at each
selections.

- First match selections. Apply a change in Normal mode, like dw. Then press `.`
to repeat the operation.

### More tips
1. **Select operator**: press `si"` in cursors mode can switch to extern mode and select inside the quotes.
1. **Replace in regions**: press `R` to replace a pattern in each region.
1. **Toggle case**: press `~` to toggle case for character  under cursors.
1. **Numbering**: create multiple cursors, pressing `\\N` will insert numbering
ahead of each cursors(`\\n` will insert after cursors).
1. **\<C-v\> operation**: you can using \<C-v\> in Insert mode to paste deleted
contents in VM.



