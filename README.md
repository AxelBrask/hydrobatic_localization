# Hydrobatic Localizaiton
This project is a localization scheme implemented with the GTSAM framework for the AUV SAM, the project contains custom factors for the Doppler Velocity Logger, Barometer and the Motion Model, acounting for the offset of the sensors with respect to the base link. In order to build the project the following needs to be installed:
* GTSAM
* Geographiclib for the lat/lon to UTM converions: https://geographiclib.sourceforge.io/C++/doc/install.html
* Follow the instrucitons and build the smarc_modelling submodule in order to run the motion model.

In oder to run the localizer simply run the launch file, this will use the default parameters.
```
ros2 launch hydrobatic_localization state_estimator.launch 
```
The launch file has 2 variables that can be specified, namely a boolean **use_motion_model**, which specifies if the ros node should subscribe to the contorl inputs and the gtsam graph should add the motion model factor to the graph. The deafault is set to **true**. The other parameter is **inference_strategy**, which specifies if the gtsam graph should use *fullsmoothing*, *ISAM2* or *fixedlagsmoothining*, the flags are **FullSmoothing**, **ISAM2** and **FixedLagSmoothing** respectivly, with fullsmoothing being the default.

A example for running with the motion model turned off and with the fixedlagsmoother is:
```
ros2 launch hydrobatic_localization state_estimator.launch use_motion_model:=false inference_strategy:=FixedLagSmoother

```
