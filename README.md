# python binding for leadshine smc motion controller

python binding for leadshine smc motion controller by [pybind11](https://github.com/pybind/pybind11). 

![SMC604](http://www.szleadtech.com.cn/upload/201504/23/201504231130000706.jpg)

## Prerequisites

- Linux system
- A compiler with C++11 support
- Pip 10+ or CMake >= 3.4


## Building
Only can be used in linux system (Tested in Ubuntu 20.04 and leadshine smc604).

```
git clone https://github.com/jsbyysheng/pyltsmc.git
git submodule update --init
mkdir build && cd build
cmake ..
make
```

## Statement

This project can only be used for science research. 
The Leadshine company and `pyltsmc` authors will take no responsibility for any results by `pyltsmc`. 

## License

`pyltsmc` is provided under a BSD-style license that can be found in the LICENSE
file. By using, distributing, or contributing to this project, you agree to the
terms and conditions of this license.
