---
title: 'How to use codna env under ROS noetic and, python and ROS '
date: 2024/12/20
permalink: /posts/2024/12/blog-post-ROS-conda/
tags:
  - ROS
  - Conda
---

ROS 在编译的时候应该默认的是系统自带的python 环境。

ROS 的依赖应该是这样的，ROS 用的编译的是 环境中的package. python 算ROS编译时候 packages 依赖里面的一个，ROS编译默认使用系统自带的python环境。

如果我们使用pip install 的packages算是 python里面的 packages里面的一种。

首先需要明确：catkin_make 时指定的 PYTHON_EXECUTABLE 会决定工作空间中生成的配置（比如 CMake 配置和 Python 包路径），

**!!如果在编译时指定了特定的 Python 环境（比如 Anaconda 或系统的 /usr/bin/python3），运行时通常也会使用这个环境中的 Python。!!**

所以如果我们想使用conda 虚拟的python环境，在ROS里面使用，需要让ROS 指定python 环境为conda 虚拟的python环境

```
catkin_make -DPYTHON_EXECUTABLE=/path/to/desired/python
```

这样的catkin编译时候就会指定` $  DPYTHON_EXECUTABLE=/path/to/desired/python $ `作为编译环境！

注意catkin_make如果不修改，就会一直指定DPYTHON_EXECUTABLE=/path/to/desired/python 作为编译环境，如果想要修改需要重新使用`catkin_make -DPYTHON_EXECUTABLE=/path/to/desired/python`

编译环境和运行环境测试一下，

如果我首先使用系统base环境编译,使用 `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`,

即使我在终端下切换到了conda 虚拟环境，这时候rosrun，我们可以发现rosrun 指定的python 还是系统自带的python文件

catkin_make一下发现：

```
(main) tianyi@tianyi-Redmi-G-2022:~/test_ws_conda$ catkin_make
Base path: /home/tianyi/test_ws_conda
Source space: /home/tianyi/test_ws_conda/src
Build space: /home/tianyi/test_ws_conda/build
Devel space: /home/tianyi/test_ws_conda/devel
Install space: /home/tianyi/test_ws_conda/install
####
#### Running command: "make cmake_check_build_system" in "/home/tianyi/test_ws_conda/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/tianyi/test_ws_conda/devel
-- Using CMAKE_PREFIX_PATH: /home/tianyi/test_ws_conda/devel;/home/tianyi/aruco/devel;/home/tianyi/pose_estimation/devel;/opt/ros/noetic
-- This workspace overlays: /home/tianyi/test_ws_conda/devel;/home/tianyi/aruco/devel;/home/tianyi/pose_estimation/devel;/opt/ros/noetic
-- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3") 
-- Using PYTHON_EXECUTABLE: /usr/bin/python3
-- Using Debian Python package layout
-- Using empy: /usr/lib/python3/dist-packages/em.py
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/tianyi/test_ws_conda/build/test_results
-- Forcing gtest/gmock from source, though one was otherwise available.
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python3 (found version "3.8.10") 
-- Using Python nosetests: /usr/bin/nosetests3
-- catkin 0.8.10
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
```

说明即使在虚拟环境里使用catkin_make，系统还是会选择上一次 explicitly defined的python_executable进行编译。
所以如果需要使用conda环境，只需要`catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`,就好了，以后编译还会使用之前explicitly defined的python 编译器

![image](https://github.com/user-attachments/assets/5219f035-1999-41ca-90de-6d0eff18646c)

最后附带一下我的电脑的所有python环境，/usr/bin/python3 是系统自带默认的环境，其余的是annoconda 安装后的环境，注意这里面annoconda base 不是系统/usr/bin/pyhon3，我们每次启动annoconda 
被带进的环境其实是/anaconda3/bin/python，如果需要使用系统usr/环境，需要deactivate conda

