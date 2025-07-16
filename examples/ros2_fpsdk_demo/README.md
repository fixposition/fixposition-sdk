# Fixposition SDK: ROS2 Demo

An example ROS2 package with a node demonstrating the use of the Fixpositon SDK.

---
## Dependencies

- [Fixposition SDK dependencies](../../README.md)
- [fpsdk_common](../../fpsdk_common/README.md)
- [fpsdk_ros2](../../fpsdk_ros2/README.md)


---
## Build

Setup a new ROS2 workspace and build the demo and its dependencies:

```sh
mkdir -p ws/src
cd ws/src
ln -s ../../fpsdk_common .
ln -s ../../fpsdk_ros2 .
ln -s ../../examples/ros2_fpsdk_demo .
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

To run the node:

```sh
source install/setup.bash
ros2 launch ros2_fpsdk_demo node.launch
```

---
## License

See the [LICENSE](LICENSE) file and [../fpsdk_doc/README.md#license](../fpsdk_doc/README.md#license).
