# Fixposition SDK: Apps

This contains various apps (tools, utilities). See below for details.

---
## Dependencies

- [Fixposition SDK dependencies](../fpsdk_doc/README.md#dependencies)
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

See the documentation for available apps: [doc/doc.hpp](doc/doc.hpp) or the
[generated documentation](https://fixposition.github.io/fixposition-sdk/fixposition-sdk-docs/FPSDK_APPS_DOC.html).


---
## License

See the [LICENSE](LICENSE) file and [../fpsdk_doc/README.md#license](../fpsdk_doc/README.md#license).
