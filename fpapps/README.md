# Fixposition SDK: Apps

This contains various apps (tools, utilities). See below for details.

---
## Dependencies

- [Fixposition SDK dependencies](../README.md#dependencies)
- [fpcommon](../fpcommon/README.md)
- [fpros1](../fpcommon/README.md) -- optional, but recommended, and required for some functionality


---
## Build

```sh
source /opt/ros/noetic/setup.bash # optional, but recommoneded, and required for ROS functionalities
cmake -B fpapps/build -S fpapps -DCMAKE_INSTALL_PREFIX=~/fpsdk
make -C fpapps/build
make -C fpapps/build install
```

Or in a ROS workspace:

```sh
catkin build fpapps
```

See also [Fixposition SDK building](../README.md#building)


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
