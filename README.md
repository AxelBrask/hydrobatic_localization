# Hydrobatic Localization
This project is a localization scheme implemented with the GTSAM framework for the AUV SAM, the project contains custom factors for the Doppler Velocity Logger, Barometer and the Motion Model, acounting for therotation and offset of the sensors with respect to the base link. In order to build the project the following needs to be installed:
* GTSAM
* Geographiclib for the lat/lon to UTM converions: https://geographiclib.sourceforge.io/C++/doc/install.html
* Follow the instrucitons and build the smarc_modelling submodule in order to run the motion model.

In oder to run the localizer simply run the launch file, this will use the default parameters.
```
ros2 launch hydrobatic_localization state_estimator.launch 
```

The launch files have mutiple ros parameters that can be specificed, and are summarizes by the table below:
| Parameter                | Default Value       | Description                                                                                               |
| ------------------------ | ------------------- | --------------------------------------------------------------------------------------------------------- |
| `config_file`            | `sam.yaml`          | Path to the YAML configuration file that contains extrincis, noise models, and preintegration setttings         |
| `robot_name`             | `sam`               | Namespace or identifier for the robot; used to topics, frames, and parameters.                      |
| `use_motion_model`       | `false`             | Whether to include a motion  model in the state estimation               |
| `inference_strategy`     | `FixedLagSmoothing` | Choice of inference algorithm; e.g., `FixedLagSmoothing`, `Filter`, `iSAM2` and `Fullsmoothing` for state estimation.       |
| `use_sim_time`           | `false`             | If true, nodes will subscribe to the `/clock` topic instead of wall time |
| `use_sensor_covariance`  | `false`             | Whether to use the covariance filed in the messages for the dvl and gps, instead of the config file.
| `init_from_ground_truth` | `false`             | If true, initialize the estimator’s state directly from the ground truth using MoCap  |
| `robot_urdf`             | `sam_auv.urdf`      | File name or path of the robot’s URDF description, used for the robot state publisher.                 |
|`kf_interval_hz`          | `10`                | Sets the rate for the keyframe optimizations
| `frame_suffix`           | empty string        | Parameter that sets a suffix for the odom and base_link transforms, used in rosbags where the MoCap is sending out base_link and odom transfroms so there are no conflicts. The logger node uses the same frame suffix.



This is an example of launching the state estimator without the motion model and with the fixedlagsmoother:
```
ros2 launch hydrobatic_localization state_estimator.launch robot_name:=sam  use_motion_model:=false inference_strategy:=FixedLagSmoothing kf_interval_hz:=10 use_sensor_covariance:=false 
```
## Running a demo

In order to run the localization framework using a rosbag you need to run the following launch file:
```
ros2 launch hydrobatic_localization localize_bag.launch 
```
The launch file contains three more ros parameters and they are the following:
| Parameter                | Default Value       | Description                                                                                               |
| ------------------------ | ------------------- | --------------------------------------------------------------------------------------------------------- |
| `bag_file`            | -          | Path to the ros bag         |
| `enable_logger`             | `false`               | Enables the logger node to write the ground truth and the estimated pose to a csv file, this requires the MoCap ground truth. Everything in the csv will be in ENU                      |
| `folder`       | `results`             | The folder name of where the csv file will be.    |


An example of running the localization framework, logging the pose and using the MoCap to start the graph is the following:
```
ros2 launch hydrobatic_localization localize_bag.launch bag_file:=/nice/path/to/bag enable_logger:=true use_sim_time:=true init_from_ground_truth:=true frame_suffix:=gtsam
```
> **Note:** Most final tests where done using ros bag rosbag2_2025_07_03-14_52_42 with a start offset of 10 seconds.
