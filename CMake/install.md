# Install Command

* CMake doesn't allow to install `IMPORTED` libraries as `TARGETS.` Use `install(FILES)` instead.

- Define install rules.
    * TARGETS:
    ```cmake
    install(TARGETS targets...
        [[ARCHIVE|LIBRARY|RUNTIME]
        [DESTINATION <dir>]
        [PERMISSIONS permissions...]
        [CONFIGURATIONS [Debug|Release|...]]
        [COMPONENT <component>]
        [OPTIONAL]
        ] [...])
    ```
    **targets** 包含 ARCHIVE:静态库，LIBRARY:动态库， RUNTIME:可执行目标二进制, 由`add_executable`或`add_library`所定义，e.g.
    ```cmake
    install(TARGETS myrun mylib mystaticlib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION libstatic
        )
    ```
    * FILES|PROGRAMS:
    ```cmake
    install(FILES|PROGRAMS files... DESTINATION <dir>
        [PERMISSIONS permissions...]
        [CONFIGURATIONS [Debug|Release|...]]
        [COMPONENT <component>]
        [RENAME <name>] [OPTIONAL])
    ```
    * DIRECTORY:
    ```cmake
    install(DIRECOTY dirs... DESTINATION <dir>
        [PERMISSIONS permissions...]
        [CONFIGURATIONS [Debug|Release|...]]
        [COMPONENT <component>]
        [RENAME <name>] [OPTIONAL])
    ```
    Set `dirs` to `dirs` or `dirs/` will have different effect where the directory will be copied to `<dir>` without `/`, copy the contents if with `/`.

## Reference
[cmake - Can I install shared imported library? - Stack Overflow](https://stackoverflow.com/questions/41175354/can-i-install-shared-imported-library)
