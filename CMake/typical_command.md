# Overview
1. 通过`add_executable`, `add_library`, 指定要生成的`target`
2. `set_target_properties`设置`target`的属性
3. 对于`target`所依赖的库, 通过`include_directories`添加头文件，`target_link_libraries`添加库文件
    - 添加库的五种方式：
        1. `target_link_libraries(target_name, lib_name)`, `lib_name`是绝对路径或者在`/usr/lib`下能找到。
        2. `link_directories(${LIB_DIR})`再`target_link_libraries(target_name, lib_name)`, `lib_name`是相对路径。
        3. `find_path/find_library`找到库的绝对路径，然后用`target_link_libraries`同1。
        4. `find_path/find_library`里不写具体路径，而通过设置环境变量`export CMAKE_INCLUDE_PATH=include_dir, CMAKE_LIBRARY_PATH=lib_dir`传入, 其他同3。
        5. 将`Find<name>.cmake`模块路径加到系统变量`CMAKE_MODULE_PATH`中, 再通过`find_package/include`载入和运行此模块得到库路径, 其他同上
4. `install`时的安装规则

[[toc]]

## 指定要生成的target
[targets](targets.md) 包含 ARCHIVE:静态库，LIBRARY:动态库， RUNTIME:可执行目标二进制, 由`add_executable`或`add_library`所定义.
1. add_executable
```cmake
add_executable(<name> IMPORTED [GLOBAL])
```

2. add_library
```cmake
add_library(libname [SHARED|STATIC|MODULE]
    [EXCLUDE_FROM_ALL]
    source1 source2 ... sourceN)
```
* MODULE: 在使用dyld的系统有效，如果不支持dyld，则被当作SHARED。
* EXCLUDE_FROM_ALL: 这个库不会被默认构建，除非有其他的组件依赖或者手工构建。

3. add_subdirectory
向当前工程添加存放源文件的子目录(将子目录CMakeLists.txt中的内容也添加进来)。
```cmake
add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
```
* binary_dir: 二进制存放的位置。
* EXCLUDE_FROM_ALL: 将这个目录从编译过程中排除。
* 也可以通过set指令重新定义`EXECUTABLE_OUTPUT_PATH`和`LIBRARY_OUTPUT_PATH`变量来指定生成的`target`二进制位置, 在有 `add_executable`或者`add_library`处添加, 写在外面可能会无效。
    ```cmake
    # 不写`${PROJECT_BINARY_DIR}/` 会放在binary_dir下面
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
    ```

## 设置target属性
1. set_target_properties
```cmake
set_target_properties(target1 target2 ...
    PROPERTIES prop1 value1 
    prop2 value2 ...)
```

e.g. 设置输出的名称, 版本号：
```cmake
set_target_properties(hello_static PROPERTIES OUTPUT_NAME hello)
set_target_properties(hello PROPERTIES VERSION 1.2 SOVERSION 1)
```

2. get_target_property： 得到 **target** 的property, 未定义时返回NOTFOUND
```cmake
get_target_property(VAR target property)
```
e.g.
```cmake
get_target_property(OUTPUT_VALUE hello_static OUTPUT_NAME)
message(STATUS "This is the hello_static OUTPUT_NAME: ${OUTPUT_VALUE}")
```

## 为target添加共享库
1. 添加头文件目录: include_directories
```cmake
include_directories([AFTER|BEFORE] [SYSTEM] dir1 dir2)
#也可以通过CMAKE_INCLUDE_DIRECTORIES_BEFORE变量将路径加到已有路径前（默认是后）
set(CMAKE_INCLUDE_DIRECTORIES_BEFORE on)
```

2. 添加库文件
    1. `target_link_libraries`用绝对路径
    ```cmake
    #为target添加需要链接的共享库
    target_link_libraries(target library1 <debug | optimized> library2 ...)
    ```

    2. 用`link_directories/target_link_directories`添加库路径，再`target_link_libraries`用相对路径
    ```cmake
    #添加非标准共享库搜索路径
    link_directories([AFTER|BEFORE] directory1 directory2)
    target_link_directories(<target> [BEFORE] 
    <INTERFACE|PUBLIC|PRIVATE> [items1...]
    <INTERFACE|PUBLIC|PRIVATE> [items1...] ...])
    ```
    e.g.
    ```cmake
    link_directories(${LIB_DIR})
    target_link_libraries(main hello)
    target_link_libraries(main libhello.so)
    ```

    3. `find_library/find_path`找到库路径，再用`target_link_libraries`同1
    ```cmake
    find_library(<VAR> name1 path1 path2 ...)
    target_link_directories(<target> ${VAR})
    ```
    e.g.
    ```cmake
    find_library(LIB_PATH hello ../ch3_library/install)
    target_link_directories(test ${LIB_PATH})
    ```

    4. `find_path/find_library`里不写具体路径，而通过设置环境变量`export CMAKE_INCLUDE_PATH=include_dir, CMAKE_LIBRARY_PATH=lib_dir`传入, 其他同3。

    5. `find_package/include`, 载入和运行`Find<name>.cmake`模块来得到`<name>_FOUND`, `<name>_INCLUDE_DIR/<name>_INCLUDES`, `<name>_LIBRARY/<name>_LIBRARIES`等变量的值, 再通过这些变量来控制编译流程。对于非系统库要自己写`Find<name>.cmake`文件，在里面设置以上变量的值，同时将此文件路径加入`CMAKE_MODULE_PATH`变量中。
    ```cmake
    # 对于系统库
    find_package(curl)
    # 对于自定义库
    set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
    find_package(hello)
    # or
    # include(Findhello)

    if(curl_FOUND)
        include_directories(${CURL_INCLUDE_DIR})
        target_link_libraries(curltest ${CURL_LIBRARY})
    else(curl_FOUND)
        message(FATAL_ERROR "curl library not found.")
    endif(curl_FOUND)
    ```

## install规则
1. CMakeLists.txt内部： [install](install.md)
2. CMakeLists.txt外部
    ```cmake
    make install DESTDIR=install_dir
    ```
    ```cmake
    # 在CMakeLists.txt里set(CMAKE_INSTALL_PREFIX install_dir)是一样的效果
    cmake -DCMAKE_INSTALL_PREFIX=install_dir ..
    ```
    ```shell
    ./configure -prefix=install_dir
    ```

## 其他指令 
1. message
```cmake
message([SEND_ERROR|STATUS|FATAL_ERROR] "message to display" ...)
```
* SEND_ERROR: 产生错误，生成过程被跳过。
* STATUS: 输出前缀为-的信息。
* FATAL_ERROR: 立即终止所有cmake过程。

2. set
```cmake
set(<VAR> [VALUE])
```
通过`${VAR}`引用，但是在`if`语句中直接使用变量名而非`${VAR}`, e.g.
```cmake
set(LIB_DIR ${PROJECT_SOURCE_DIR}/../ch3_library/install)
if(LIB_DIR)
    message(STATUS "LIB_DIR is set to ${LIB_DIR}")
endif(LIB_DIR)
```

3. aux_source_directory: 发现`dir`下所有的源文件，并将列表存于`variable`
```cmake
aux_source_directory(dir variable)
```

4. add_definitions: 向c/c++编译器添加-D定义，类似于加了cmake变量（代码里好像也能用）
```cmake
add_definitions(-DVAR=1 -DENABLE_DEBUG)
```

5. exec_program: 在CMakeLists.txt处理过程中执行命令, 不会在生成的Makefile中执行
```cmake
exec_program(Exectuable [directory in which to run]
    [ARGS <arguments to exectuable>]
    [OUTPUT_VARIABLE <var>]
    [RETURN_VALUE] <var>])
```
在指定的目录运行某个程序，通过ARGS添加参数，通过`OUTPUT_VARIABLE`和`RETURN_VALUE`将输出和返回值赋予变量。

6. file: 文件操作指令
```cmake
file(WRITE filename "message to write"...)
file(APPEND filename "message to write"...)
file(READ filename variable)
file(GLOB variable [RELATIVE path] [globbing expressions]...)
file(GLOB_RECURSE variable [RELATIVE path] [globbing expressions]...)
file(REMOVE [directory]...)
file(REMOVE_RECURSE [directory]...)
```

7. include: 载入和运行CMakeLists.txt文件或者预定义的cmake模块. 对于module, 类似于`find_package`, 在CMAKE_MODULE_PATH中搜索这个模块并载入, 但是`module`要写`Find<name>`, e.g., `include(Findhello)`
```cmake
include(<file|module> [OPTIONAL])
```

8. list
```cmake
list(APPEND <list> [<element>...])
list(INSERT <list> <index> [<element>...])
list(REMOVE_ITEM <list> <value>...)
list(REMOVE_AT <list> <index>...)
```

## cmake常用变量
1. `CMAKE_BINARY_DIR`, `PROJECT_BINARY_DIR`, `<projectname>_BINARY_DIR`都指向工程编译发生的目录（通常是build).
2. `CMAKE_SOURCE_DIR`, `PROJECT_SOURCE_DIR`, `<projectname>_SOURCE_DIR`都指向工程顶层目录.
3. `CMAKE_CURRENT_SOURCE_DIR`：当前CMakeLists.txt文件所在目录. `CMAKE_CURRENT_LIST_FILE`: 当前CMakeLists.txt的完整路径
4. `CMAKE_CURRENT_BINARY_DIR`: out-of-source编译时指向`target`的编译目录, `add_subdirectory(src bin)`会改变其到`bin`目录, 但`set(EXECUTABLE_OUTPUT_PATH new_path)`则不会, 它仅修改了最终目标文件存放路径.
5. `CMAKE_MODULE_PATH`: 用来设置自己的cmake模块所在的路径
6. `EXECUTABLE_OUTPUT_PATH`, `LIBRARY_OUTPUT_PATH`: 用了重新定义最终结果的存放目录

### 开关选项
1. `CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS`: 来控制`if else`的书写方式
2. `BUILD_SHARED_LIBS`: 控制默认的库编译方式，`add_library`中没有指定库类型时默认编译静态库，通过`set(BUILD_SHARED_LIBS ON)`设置为默认编译动态库
3. `CMAKE_C_FLAGS`, `CMAKE_CXX_FLAGS`：设置C/C++编译选项，也可通过`add_definitions`添加

### 系统环境变量
1. 调用系统环境变量
```cmake
$ENV{VAR}
```
2. 设置系统环境变量
```cmake
set(ENV{var_name} value)
```
主要是`CMAKE_INCLUDE_PATH`和`CMAKE_LIBRARY_PATH`有作用，直接为`find_`的指令所使用
