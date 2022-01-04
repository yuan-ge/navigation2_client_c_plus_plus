# navigation2_client_c++
这是一个使用c++写的可以使用代码运行的navigation2客户端，使用的是ros-foxy版本的系统。
在官网只找到了Python的API于是自己仿照那个API和一些关于action的教程魔改了一个出来。

具体使用：
```bash
colcon build
. install/setup.bash
ros2 run mynav action-client
```