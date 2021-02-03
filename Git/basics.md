# Basic Commands

## Config & Help
* 安装完成后，第一步需要设置名字和邮箱  
```shell
git config --global user.name "<your name>"
git config --global user.email "<your email>"
```
* `git config --global http.proxy "<http://url:port>"` 来设置http代理
* `git config --global color.ui` 能设置颜色
* `git help <verb>` 或 `git <verb> --help` 或 `man git-<verb>` 来列出对应动作的帮助信息
* `git <verb> -h` 可以获得更简明的帮助输出

## Initialize repository
* `git init` 初始化本地版本库
* `git clone <url> [<where to clone>]` 克隆已有的版本库
* `git clone --depth <n> <url> [<where to clone>]` 克隆已有的版本库，并截取最近n次commits
* `git clone --branch / -b <branch name> <url> [<where to clone>]` 克隆某个分支

## Three basic concepts
Git has three basic concepts or area to store information
* __Working directory__: untracked files
* __Staging area__:
* __Repository__:

**Transition:**

Working directory -> Null
* `git restore <file>, git checkout <file>, git checkout -- <file>` 丢弃工作区的修改

Working directory -> Staging area
* `git add <file>` 跟踪新文件/把已跟踪的文件放到暂存区/合并时把有冲突的文件标记为已解决状态
* `git add -A` add, modify, and remove index entries to match the working tree 
* `git add -p, git add --patch` 添加一块块的修改
    * `y` stage this hunk
    * `n` do not stage this hunk
    * `q` quit; do not stage this hunk or any of the remaining ones
    * `a` stage this hunk and all later hunks in this file
    * `d` do not stage this hunk or any of the later hunks in this file
    * `e` manually edit the current hunk in vim
    * `?` print help
    * `s` split this hunk, only available when current hunk has an unchanged line between edits

[Reference for git-add-patch](https://levelup.gitconnected.com/staging-commits-with-git-add-patch-1eb18849aedb)

Staging area -> Working directory
* `git reset <file>` 不缓存文件
* `git restore --staged <file>` 同上
* `git reset` 不缓存所有文件

Staging area -> Repository
* `git commit -m <message>` 提交更新
* `git commit` 会打开一个文本编辑器让你输入commit message
* `git commit -v` 会打开文本编辑器, 同时详细的输出所有staging area (即将commit的修改) 中文件的diff情况
* `git commit -a` 跳过使用暂存区域, 把所有已跟踪的文件暂存起来一并提交

![State Transition](https://gitee.com/againxx/image-storage/raw/master/images/20201219095340.png =750x)

## What is index?

## Remote repository
* `git remote add <remote_name> git@github.com:againxx/***.git` 将一个已有的本地仓库和远程仓库关联，其中origin是git默认的远程仓库叫法
* `git remove rm <remote_name>` 删除远程仓库
* `git remote -v` 列出远程仓库的信息
* `git remote set-url <remote_name> git@github.com:againxx/***.git` 将远程库改为SSH连接方式, 每次不需要输入密码
* `git remote set-url <remote_name> https://github.com/againxx/***.git` 将远程库改为HTTPS连接方式, 每次需要输入密码
