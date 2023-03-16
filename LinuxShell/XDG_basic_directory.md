# XDG Basic Directory

## User directories
* __XDG_CONFIG_HOME__
    - Where user-specific configurations should be written (analogous to `/etc`).
    - Should default to `$HOME/.config`.
* __XDG_CACHE_HOME__
    - Where user-specific non-essential (cached) data should be written (analogous to `/var/cache`).
    - Should default to `$HOME/.cache`.
* __XDG_DATA_HOME__
    - Where user-specific data files should be written (analogous to `/usr/share`).
    - Should default to `$HOME/.local/share`.
* __XDG_STATE_HOME__
    - Where user-specific state files should be written (analogous to `/var/lib`).
    - Should default to `$HOME/.local/state`.

## System directories
* __XDG_DATA_DIRS__
    - List of directories separated by `:` (analogous to `PATH`).
    - Should default to `/usr/local/share:/usr/share`.
* __XDG_CONFIG_DIRS__
    - List of directories separated by `:` (analogous to `PATH`).
    - Should default to `/etc/xdg`.

## Common path of configuration file for different applications
[common cofig](https://wiki.archlinux.org/title/XDG_Base_Directory)
