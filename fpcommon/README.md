# Fixposition SDK: Common Library

This library contains various common functionality. This is used for example by the [fpapps](../fpapps/README).


---
## Dependencies

- [Fixposition SDK dependencies](../README.md#dependencies)
- This does *not* need any ROS


---
## Build

```sh
cmake -B fpcommon/build -S fpcommon -DCMAKE_INSTALL_PREFIX=~/fpsdk
make -C fpcommon/build
make -C fpcommon/build install
```

Or in a ROS workspace:

```sh
catkin build fpcommon
```

See also [Fixposition SDK building](../README.md#building)


---
## License

MIT, see [LICENSE](LICENSE)
