This project is a ROS2 package that contains the entrypoint launch files of the FIRA hackathon.

## Composition of this project

This package contains the following directory:

* `cfg_chal1` contains the configuration files used by the _challenge 1_ launch file
* `cfg_chal2` contains the configuration files used by the _challenge 2_ launch file
* `cfg_chal1` contains the configuration files used by the _challenge 3_ launch file
* `demo` contains the configuration files used by the _demo_ launch file
* `launch` contains the launch file (described below)
* `rviz` contains the rviz2 configuration

The directories `cfg_xxx` contains several YAML files used by the different nodes.
This files contains ROS parameter of the nodes and other parameters that affect the behavior of
launch files.
More details about these configuration files can be found in [the documentation of the tirrex
workspace](https://github.com/Tirrex-Roboterrium/tirrex_demo/blob/main/doc/documentation.pdf).

### Composition of a configuration directory

Here is the files you can find at the root of each `cfg_xxx` directory:

* `paths`: directory containing `.traj` that can be used as path for robot to follow.
* `robots`: directory containing a sub-directory for each robot or vehicle
* `localisation.yaml`: configuration of the localisation nodes
* `records.yaml`: configuration of records using ros2 bag
* `simulation.yaml`: configuration of the gazebo world used for the simulation
* `wgs84_anchor.yaml`: (unused) geographic coordinates of the origin point used to define the _map_
frame

The sub-directories of `robots` contain:

* `devices`: directory containing the configuration of the sensors and other elements of the robot
* `base.yaml`: configuration of the robot model and the spawn position in the simulation
* `devices.yaml`: allows to add/remove devices on the robot
* `path_following.yaml`: configuration of the path following node
* `path_matching.yaml`: configuration of the path matching node and the selected trajectory to
follow
* `teleop.yaml`: configuration of the manual control of the robot (using a joypad)


## Running the demo and challenges

There is 4 main launch files that can be run directly from the command line:

* [`demo.launch.py`](launch/demo.launch.py) starts a demo with the simulator and 2 robots following
the same trajectory.
* [`challenge1.launch.py`](launch/challenge1.launch.py) starts the simulator with a unique robot
following a trajectory.
* [`challenge2.launch.py`](launch/challenge2.launch.py) starts the simulator with a unique robot
following a trajectory. The farm environment contains several obstacles near the trajectory of
the robot.
* [`challenge3.launch.py`](launch/challenge3.launch.py) starts the simulator with a unique robot
following a trajectory.
The environment also contains obstacles but this time, they block the trajectory and the robot must
avoid them.

### description of launch files

The composition of this 4 launch is similar.
They do the following actions:

* start the simulator, by including the launch file `simulator.launch.py` of the `tirrex_demo` package
* spawn the robot, by including the launch file `robot.launch.py` of this package (described below)
* optionally spawn other vehicles, if the challenge requires dynamic obstacles
* start `rviz2`, to visualize the robot localization and its trajectory
* start `rqt_runtime_monitor`, to check the health of some ROS nodes

To spawn a robot, the [`robot.launch.py`](launch/robot.launch.py) file do the following actions:

* generate the URDF file and spawn the gazebo robot model.
* start localization nodes, used to publish an Odometry topic merging odometry, GNSS and IMU data.
* start the path matching node, used to compute lateral ang angular distance to the trajectory to
follow.
* start the path following node, used to compute command messages for the robot based on the result
of the path matching node.
This node corresponds to the
[`fira_minimal_node` project](https://github.com/FiraHackathon/fira_minimal_node/tree/main).

For the challenge 3, you have to develop your own version or modify the existing `fira_minimal_node`.
