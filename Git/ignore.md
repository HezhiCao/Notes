# Ignore

* 子目录也可以拥有`.gitignore`文件，并且比全局`.gitignore`文件拥有更高的优先级
* 在子目录`.gitignore`文件里，可以通过`!<pattern>`来否定之前的ignore
* `.gitignore`文件中可以使用`#`来注释
* `git ls-files --others --ignored --exclude-standard` 展现目前git ignore的文件

## example
```gitignore
# 忽略所有的 .a 文件
*.a

# 但跟踪所有的 lib.a，即便你在前面忽略了 .a 文件
!lib.a

# 只忽略当前目录下的 TODO 文件，而不忽略 subdir/TODO
/TODO

# 忽略任何目录下名为 build 的文件夹
build/

# 忽略 doc/notes.txt，但不忽略 doc/server/arch.txt
doc/*.txt

# 忽略 doc/ 目录及其所有子目录下的 .pdf 文件
doc/**/*.pdf
```

