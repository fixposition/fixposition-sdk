# Fixposition SDK: ROS1 Demo

An example ROS1 package with a node demonstrating the use of the Fixpositon SDK.

---
## Dependencies

- [Fixposition SDK dependencies](../../README.md)
- [fpsdk_common](../../fpsdk_common/README.md)
- [fpsdk_ros1](../../fpsdk_ros1/README.md)


---
## Build

Setup a new ROS1 workspace and build the demo and its dependencies:

```sh
mkdir -p ws/src
cd ws/src
ln -s ../../fpsdk_common .
ln -s ../../fpsdk_ros1 .
ln -s ../../examples/ros1_fpsdk_demo .
cd ..
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
catkin build
```

To run the node:

```sh
source devel/setup.bash
roslaunch ros1_fpsdk_demo node.launch
```

---
## License

See the [LICENSE](LICENSE) file and [../fpsdk_doc/README.md#license](../fpsdk_doc/README.md#license).
