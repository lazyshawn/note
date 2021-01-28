## § Introduction to UltiSnips
> [GitHub -- UltiSnips](
https://github.com/SirVer/ultisnips)

### I. Basic Snippets
Prototype for basic snippets: 
```vim
snippet <trigger> "commit" [option]
${1:default} foo $2;
bar;
$0
endsnippet
```

The 'options' control the behavior of the snippet. Options are indicated by
single characters. The 'options' characters for a snippet are combined into
a word without spaces.

The options currently supported are:
* **b** : Begin of a line. || A snippet with this option is expanded only if
the tab trigger is the first word on the line.

* **i** : In-word expansion. || By default a snippet is expanded only if the 
tab trigger is the first word on the line or is preceded by one or more 
whitespace characters.

* **w** : Word boundary. || The snippet is expanded if the tab trigger start
matches a word boundary and the tab trigger end matches a word boundary.

* **r** : Regular expression. || 

* **t** : Do not expand tabs. || If this option is set, UltiSnips will ignore 
the Vim settings and insert the tab characters as is, instead of replace it 
wit space.

* **s** : Remove whitespace immediately before the cursor at the end of a 
line before jumping to the next tabstop. || This is useful if there is a
tabstop with optional text at the end of a line.

* **m** : Trim all whitespaces from right side of snippet lines. || Useful 
when snippet contains empty lines which should remain empty after expanding.
Without this option empty lines in snippets definition will have indentation
too.

* **e** : Custom context snippet. || With this option expansion of snippet
can be controlled not only by previous characters in line, but by any given
python expression.

* **A** : Snippet will be triggered automatically, when condition matches.


### II. Placeholder
#### 1. Basic placeholder



#### 2. VISUAL
<++>
Snippets can contain a special placeholder called ${VISUAL}. The ${VISUAL}
variable is expanded with the text selected just prior to expanding the
snippet.
```bash
${VISUAL:default text}
${VISUAL:inside tran1/tran2/g}
```

**Usage:** use Vim's Visual mode to select some text, and then press the
key you use to trigger expanding a snippet. The selected text is deleted,
and you are dropped into Insert mode. Now type the snippet tab trigger and
press the key to trigger expansion.


### III. Interpolation
#### 1. Shellcode
Put a shell command in a snippet and when the snippet is expanded, the shell
command is replaced by the output produced when the command is executed.
The syntax for shellcode is simple: wrap the code in backticks, '\`'.
```bash
`date +%d.%m.%y`
```

#### 2. Vimcode(VimL)
Wrap the code in backticks and to distinguish it as a Vim script, start the
code with '!v'.
```bash
`!v indent(".")`
```

#### 3. Python
Wrap the code in backticks and start with '!p'. Python scripts can be run
using the python shebang(#!) '#!/usr/bin/python', but using the '!p' format
comes with some predefined objects and variables, like 'snip'. With python
code the value of the 'snip.rv' property replaces the code. Standard output
is ignored.

The variables automatically defined in python code are:
| variables | description                                                   |
| ----      | ---                                                           |
| fn        | The current filename                                          |
| path      | The complete path to the current file                         |
| t         | The array of the placeholder, t[i] -- ${i}                    |
| snip      | UltiSnips.TextObjects.SnippetUtil object instance.            |
| contex    | Result of context condition.                                  |
| match     | (RE) The return value of the match of the regular expression. |

The 'snip' object provides the following **methods**:

* `snip.mkline(line="", indent=None):`
Returns a line ready to be appended to the result.

* `snip.shift(amount=1):`
Shifts the default indentation level used by mkline right by the number of
spaces defined by 'shiftwidth', 'amount' times.

* `snip.unshift(amount=1):`
Shifts the default indentation level used by mkline left by the number of
spaces defined by 'shiftwidth', 'amount' times.

* `snip.reset\_indent():`
Reset the indentation level to its initial value.

* `snip.opt(var, default):`
Checks if the Vim variable 'var' has been set. If so, it returns the
variable's value; otherwise, it returns the value of 'default'.

The 'snip' object providers some **properties**:
| properties | description |
| --- | --- |
| snip.rv | Return value, which will replace the python block in the snippet. | 
| snip.c | The text currently in the python block's position within the snippet. |
| snip.v | Data related to the ${VISUAL} placeholder. (attr: mode/text)|
| snip.fn | The current filename. |
| snip.basename | The current filename with the extension removed. |
| snip.ft | The current filetype. |
| snip.p | Last selected placeholder. (attr:current_text/start/end)|

The 'snip' object providers the following **operators**:
* `snip >> amount`:
Equivalent to snip.shift(amount).
* `snip << amount`:
Equivalent to snip.unshift(amount).
* `snip += line`:
Equivalent to "snip.rv += '\n' + snip.mkline(line)".

Any variables defined in a python block can be used in other python blocks
that follow within the same snippet. Also, the python modules 'vim', 're',
'os', 'string' and 'random' are pre-imported within the scope of snippet code.



### IV. Global Snippets
Global snippets provide a way to reuse common code in multiple snippets.
```python
global !p
def function(parameter):
    foo
endglobal
```



