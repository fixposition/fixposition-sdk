# Fixposition SDK: Apps

This contains various apps (tools, utilities). See below for details.

---
## Dependencies

- [Fixposition SDK dependencies](../README.md#dependencies)
- [fpsdk_common](../fpsdk_common/README.md)
- [fpsdk_ros1](../fpsdk_common/README.md) -- optional, but recommended, and required for some functionality


---
## Build

> While this package can be built individually, it's recommended to build the entire SDK as described
> [here](../fpsdk_doc/README.md#building).

```sh
source /opt/ros/noetic/setup.bash # optional, but recommoneded, and required for ROS functionalities
cmake -B build -DCMAKE_INSTALL_PREFIX=~/fpsdk
cmake --build build
cmake --install build
```

Or in a ROS workspace:

```sh
catkin build fpsdk_apps
```


---
## Apps

### fpltool

This is a tool that can do various operations on or with logs from the Vision RTK 2 sensor (`.fpl` files).
For example, it can convert the data to a ROS bag or extract a part from long logfile.

See the built-in help (source: [fpltool_args.hpp](fpltool/fpltool_args.hpp)) for details and examples:

```sh
~/fpsdk/bin/fpltool -h
```


---
## License

MIT, see [LICENSE](LICENSE)
