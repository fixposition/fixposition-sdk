# Fixposition SDK: ROS2 Library

This library contains various ROS2 functionality. This is used for example by the [fpapps](../fpapps/README).


---
## Dependencies

- [Fixposition SDK dependencies](../README.md#dependencies)
- [fpcommon](../fpcommon/README.md)
- This *does* need ROS (Humble, Jazzy)


---
## Build

> While this package can be built individually, it's recommended to build the entire SDK as described
> [here](../README.md#building).

To build and install:

```sh
source /opt/ros/noetic/setup.bash
cmake -B build -DCMAKE_INSTALL_PREFIX=~/fpsdk
cmake --build build
cmake --install build
```

Or in a ROS workspace:

```sh
colcon build fpros2
```


---
## License

MIT, see [LICENSE](LICENSE)
