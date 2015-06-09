# Codestyle for this repository

## Introduction
This repository uses XML, C++ and Python for most of it's files. This document specifies how the code should look to ensure readibility and consistency.


### Variable Names
##### Classes
Classes use camel case (CamelCase) - they always begin with a capital letter and every new word in the name aslso has a capital letter. For example a class name which should be saying "Base class" will be called BaseClass.

##### Variables
Variables use snake case. In the snake case each word is separated by underscores ( "_" ). So, if a variable is to be named like "Important variable" it's name would be important_variable. Note that all the letters are lowercase.

##### Functions
Functions use snake case. In the snake case each word is separated by underscores ( "_" ). So, if a function is to be named like "Callback function" it's name would be callback_function. Note that all the letters are lowercase


### Newline characters
Unix newline characters `\n` are used everywhere.


### Blank lines
Use blank lines to make the code more readible. In the top level, use 2 blank lines for separation. Inside a class use 1 blank line for separation.
```
import math


def top_level_function():
    pass


class top_level_class:
    def foo(self):
        pass

    def bar(self):
        pass
```

### Imports/Includes
Imports need to be made in order of : system > ros/other modules > same module. A space needs to be kept in between each group.
```
import random

import rospy

import this_package.msg
```

### Spacing

##### Indentation

Code has to be indented with 4 spaces per indentation level for python and 2 spaces for C++and XML. Tabs are not allowed to ensure consistency on different editor configurations.

##### Multiline Statements

If you want to write a construct with brackets and elements that does not fit
into one line, use oe of these two variants.

You can align each element with the one above:

```python
foo = long_function_name(var_one,
                         var_two,
                         var_three)
```

You can align all (including the first one) arguments one
indentation level deeper relative to the indentation level of the _next_
statement; this is called hanging indent:

```python
foo = long_function_name(
    var_one,
    var_two,
    var_three,
    var_four)
```

or

```python
def long_function_name(
        var_one,
        var_two,
        var_three,
        var_four):
    if (
            var_one or
            var_two or
            var_three or
            var_four):
        do_something_great()
```

This can be used for very long (e.g. constant string) arguments.
