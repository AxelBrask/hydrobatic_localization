# Hydrobatic Localizaiton
This project is a localization scheme implemented with the GTSAM framework for the AUV SAM, the project contains custom factors for the Doppler Velocity Logger, Barometer and the Motion Model, acounting for the offset of the sensors with respect to the base link. In order to build the project the following needs to be installed:
* GTSAM
* Geographiclib for the lat/lon to UTM converions: https://geographiclib.sourceforge.io/C++/doc/install.html
* Follow the instrucitons and build the smarc_modelling submodule in order to run the motion model.

In oder to run the localizer simply run the launch file, this will use the default parameters.
```
ros2 launch hydrobatic_localization state_estimator.launch 
```
<!-- The launch file has mutiple parameters that can be specified, namely a boolean **use_motion_model**, which specifies if the ros node should subscribe to the control inputs and the gtsam graph should add the motion model factor to the graph. The deafault is set to **true**. The other parameter is **inference_strategy**, which specifies if the gtsam graph should use *fullsmoothing*, *ISAM2* or *fixedlagsmoothining*, the flags are **FullSmoothing**, **ISAM2** and **FixedLagSmoothing** respectivly, with FixedLagSmoothing being the default. The **kf_interval_hz** parameters sets the rate at which keyframes optimizations are done, and **use_sensor_covariance** specifies if the covariances from the GPS and DVL drivers should be used instead of the ones in the config file. In order to specify a config file either use the name of the file or the full path.  -->
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
|`kf_interval_hz`          | `10`                 | Sets the rate for the keyframe optimizations



This is an example of launching the state estimator without the motion model and with the fixedlagsmoother:
```
ros2 launch hydrobatic_localization state_estimator.launch robot_name:=sam  use_motion_model:=false inference_strategy:=FixedLagSmoothing kf_interval_hz:=10 use_sensor_covariance:=false 
```
There is also a parameter **init_from_ground_truth** which makes the state estimator use the ground truth from the MoCap in order to initialize the tf frames and the initial position of the state estimator. 


