# Fixposition SDK: ROS1 Library

This library contains various ROS1 functionality. This is used for example by the [fpapps](../fpapps/README).


---
## Dependencies

- [Fixposition SDK dependencies](../README.md#dependencies)
- [fpcommon](../fpcommon/README.md)
- This *does* need ROS (Noetic)


---
## Build

```sh
source /opt/ros/noetic/setup.bash
cmake -B fpros1/build -S fpros1 -DCMAKE_INSTALL_PREFIX=~/fpsdk
make -C fpros1/build
make -C fpros1/build install
```

Or in a ROS workspace:

```sh
catkin build fpros1
```

See also [Fixposition SDK building](../README.md#building)


---
## License

MIT, see [LICENSE](LICENSE)
