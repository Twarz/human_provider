<h1 align="center"> Human Perception Package </h1>

## Citations
This repository uses 3 other works, all credits go to them.

#### RT-GENE: Real-Time Eye Gaze Estimation in Natural Environments
Paper: [RT-GENE: Real-Time Eye Gaze Estimation in Natural Environments](http://openaccess.thecvf.com/content_ECCV_2018/html/Tobias_Fischer_RT-GENE_Real-Time_Eye_ECCV_2018_paper.html)

Github: [rt_gene](https://github.com/Tobias-Fischer/rt_gene)


#### OpenPose
Papers:
- [OpenPose: Realtime Multi-Person 2D Pose Estimation using Part Affinity Fields](https://arxiv.org/abs/1812.08008)
- [Realtime Multi-Person 2D Pose Estimation using Part Affinity Fields](https://arxiv.org/abs/1611.08050)
- [Hand Keypoint Detection in Single Images using Multiview Bootstrapping](https://arxiv.org/abs/1704.07809)
- [Convolutional Pose Machines](https://arxiv.org/abs/1602.00134)

(ROS and tensorflow version) Github: [tf-pose-estimation](https://github.com/ildoonet/tf-pose-estimation)

Original Github: [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose)

#### Underworlds
Paper: [UNDERWORLDS: Cascading Situation Assessment for Robots](https://academia.skadge.org/publis/lemaignan2018underworlds.pdf)

Github: [uwds](https://github.com/underworlds-robot/uwds)

## Description
This ROS package is designed to detect important parts of human body in order to improve Human Robot Interaction.
It is a simple Perception stack, based on ROS and it also provides a bridge to Underworlds ([uwds](https://github.com/underworlds-robot/uwds)).
Each detector is working with a quite fast frame rate (>20FPS), on natural environments and with many person in front of the camera.

#### Human body and gaze
Up to now, this package contains 2 detectors:
- 3D human body estimation (based on [rt_gene](https://github.com/Tobias-Fischer/rt_gene))
- 3D human gaze estimation (based on [tf-pose-estimation](https://github.com/ildoonet/tf-pose-estimation))

The default functioning of this package is to detect a 2D human skeleton based on [tf-pose-estimation](https://github.com/ildoonet/tf-pose-estimation) and then project it on depth image so we can get the 3D pose of the body. At the same time, [rt_gene](https://github.com/Tobias-Fischer/rt_gene) provides 3D head/gaze poses (using only rgb image) that we merge with the corresponding body. At the end, we have a full human body with the estimated gaze.

![](https://github.com/Twarz/human_provider/blob/master/misc/graph_human_provider.png)

###### Notes
- Only the head estimation is used from [rt_gene](https://github.com/Tobias-Fischer/rt_gene) from now. 
- Only the upper half body is used, because of table-top experiments.

#### Only human body
Only the body perception can be used regarding the experiment tasks, an head position is still estimated by [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose).

#### Only human gaze
Also, only the gaze perception can be used.

#### UWDS bridge
This package includes a bridge to connect the perception stack to [uwds](https://github.com/underworlds-robot/uwds) that extracts many beliefs.

## Installation
#### Dependencies
###### Python
###### ROS
###### RT-GENE
###### OpenPose
###### Underworlds


## Run
#### Choose parts
To run the whole human provider:``` roslaunch human_provider human_provider.launch ```

To only run the gaze provider:``` roslaunch human_provider gaze_provider.launch ```

To only run the body provider:``` roslaunch human_provider body_provider.launch ```

#### ROS-UWDS bridge
To activate the full workflow with UWDS bridge:``` roslaunch human_provider uwds_human_provider.launch ```

To activate the full workflow with UWDS bridge with a specific provider: ``` roslaunch human_provider uwds_human_provider.launch _what_to_provide:=<"only_gaze" OR "only_body" OR "both"(default)> ```

#### Vizualisation
By default the vizualisation is activated, you can get it on RVIZ, subscribing to the MarkerArray topic ``` '/humans/poses/3D/markers' ```.
To deactivate the vizualisation, use the option:``` viz:="False" ```
