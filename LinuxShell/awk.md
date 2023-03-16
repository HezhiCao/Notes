# Awk
`awk [Options] [Progrem] [file ...]`

- Specify `NR` to capture output from selected rows
`awk 'NR==1{print $1, $3}'`
    - only first row
    `awk '{print $1, $3; exit}'`

* `-F value`: set field separator to value
* `-f program-file`: read from file instead of command-line

e.g.
- 默认每行按空格或TAB分割
```
输出文本中的1、4项
awk '{print $1,$4}' log.txt
```
- 指定分隔符
`awk -F, '{print $1, $2}' log.txt`
- 使用正则，字符串匹配
```
# 输出第二列包含 "th"，并打印第二列与第四列
awk '$2 ~ /th/ {print $2,$4}' log.txt
```
- 模式取反
```
awk '$2 !~ /th/ {print $2,$4}' log.txt
```
