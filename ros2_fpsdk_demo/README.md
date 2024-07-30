# Fixposition SDK: ROS2 Demo

An example ROS2 package with a node demonstrating the use of the Fixpositon SDK.

---
## Dependencies

- [Fixposition SDK dependencies](../README.md#dependencies)
- [fpcommon](../fpcommon/README.md)
- [fpros2](../fpros2/README.md)


---
## Build

Setup a new ROS2 workspace and build the demo and its dependencies:

```sh
mkdir -p ws/src
cd ws/src
ln -s ../../fpcommon .
ln -s ../../fpros2 .
ln -s ../../ros2_fpsdk_demo .
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

To run the node:

```sh
source install/setup.bash
ros2 launch ros2_fpsdk_demo node.launch
```

---
## License

MIT, see [LICENSE](LICENSE)
