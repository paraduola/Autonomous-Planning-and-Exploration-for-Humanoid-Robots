
一、说明：

环境要求：ubuntu20.04 ros1 noetic
如果是canda环境请先退出！！conda deactivate或暂时在.bashhrc里注释掉conda的自启动。

1、编译：

 Compile and install the Livox-SDK2:
```shell
$ cd catkin_ws/Livox-SDK/
$ mkdir build
$ cd build
$ cmake .. && make -j
$ sudo make install
$ cd catkin_ws/
$ source /opt/ros/noetic/setup.bash
$ catkin_make
``

2、运行导航算法：
cd catkin_ws
source devel/setup.bash
roslaunch g1_nav fusion.launch


