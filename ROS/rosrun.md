# rosrun

`rosrun [package_name] [node_name]`
run the node inside a package directly.

Change the node name by Remapping Argument `__name:=`
`rosrun turtlesim turtlesim_node __name:=my_turtle`

```
$ rosnode list
/rosout
/my_turtle  # shows my_turtle instead of turtlesim_node

```
