# ROS package for Human Perception


## Description
This ROS package is designed to detect important parts of human body in order to improve Human Robot Interaction.
It is a simple Perception stack, based on ROS and it also provides a bridge to Underworlds (UWDS).
Each detector is working with a quite fast frame rate (>20FPS), on natural environments and with many person in front of the camera.

#### Human body and gaze
Up to now, this package contains 2 detectors:
- 3D human body estimation (based on RT_GENE)
- 3D human gaze estimation (based on OpenPose)
The default functioning of this package is to detect a 2D human squeleton and then project it on depth image so we can get the 3D pose of the body. At the same time, RT_GENE provides 3D head/gaze poses (using only rgb image) that we merge with the corresponding body. At the end, we have a full human body with the estimated gaze.

###### Notes
- Only the head estimation is used from RT_GENE from now. 
- Only the upper half body is used, because of table-top experiments.

#### Only human body
Only the body perception can be used regarding the experiment tasks, an head position is still estimated by OpenPose.

#### Only human gaze
Also, only the gaze perception can be used for regarding the experiment tasks, an head position is still estimated by OpenPose.

#### UWDS bridge
This package includes a bridge to connect the perception stack to Underworlds that extracts many beliefs.

## Installation
#### Dependencies
###### ROS
###### RT_GENE
###### OpenPose
###### Underworlds


## Run
#### Choose parts
To run the whole human provider:
```sh
roslaunch human_provider human_provider.launch
```

To only run the gaze provider:
```sh
roslaunch human_provider gaze_provider.launch
```

To only run he body provider:
```sh
roslaunch human_provider body_provider.launch
```

#### ROS-UWDS bridge
To activate the full workflow with UWDS bridge:
```sh
roslaunch human_provider uwds_human_provider.launch
```

To activate the full workflow with UWDS bridge with a specific provider:
```sh
roslaunch human_provider uwds_human_provider.launch _what_to_provide:=<"only_gaze" OR "only_body" OR "both"(default)>
```

#### Vizualisation
By default the vizualisation is activated, you can get it on RVIZ, subscribing to the MarkerArray topic '/humans/poses/3D/markers'.
To deactivate the vizualisation, use the option:
```sh
_viz:="False"
```
