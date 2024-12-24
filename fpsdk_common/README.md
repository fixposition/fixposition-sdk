# Fixposition SDK: Common Library

This library contains various common functionality. This is used for example by the [fpsdk_apps](../fpsdk_apps/README).

---
## Dependencies

- [Fixposition SDK dependencies](../fpsdk_docs/README.md#dependencies)
- This does *not* need any ROS


---
## Build

> While this package can be built individually, it's recommended to build the entire SDK as described
> [here](../fpsdk_doc/README.md#building).

To build and install:

```sh
cmake -B build -DCMAKE_INSTALL_PREFIX=~/fpsdk
cmake --build build
cmake --install build
```

Or in a ROS workspace:

```sh
catkin build fpsdk_common
```


---
## License

MIT, see [LICENSE](LICENSE)
