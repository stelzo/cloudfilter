# Cloudfilter

A simple ROS1 node that filters a `sensor_msgs/PointCloud2` based on a given filter function.

This is a good starting point for implementing ROS1 point cloud nodes (not just filters) with [rosrust](https://crates.io/crates/rosrust) inside a catkin environment. See the `CMakelists.txt` and `build.sh` for details.

Maybe this node will be extended with more complex filters in the future but for now it is just a simple example for working
with point clouds in Rust and an example for working with the `ros_pointcloud2` crate.

Note: The points get transformed into a different frame before applying the filter function.

## Features
- **Bounding Box Filter**. All bounds are infinite by default. Change them with the parameters in `filter.launch`.
- **Keep meta fields** (e.g. RGB, intensity, label). Change the types in `src/main.rs` and the point conversion in the filter function for this. See predefined types in the [ros_pointcloud2](https://docs.rs/ros_pointcloud2) docs for more.
- **Use as starting point for your own node**. `CMakelists.txt` and `build.sh` are helpful for creating other nodes.

## Usage
You do not need to have Rust in order to use this node. It will be installed automatically by the `build.sh` script.

```shell
cd ~/catkin_ws/src
git clone https://github.com/stelzo/cloudfilter
catkin build

source ../devel/setup.bash
roslaunch cloudfilter filter.launch # edit this file for your needs
```

## License
[MIT](https://choosealicense.com/licenses/mit/)
