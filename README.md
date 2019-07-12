<h1 align="center"> Human centered ROS package for <br> Human Robot Interaction </h1>

## General Dependencies
- ROS (tested on Ubuntu1804/Melodic)
- catkin
- python/cpp
- GPU (tested on GeForce 1050 for only one perception module)
- RGB-D camera (in order to compute 3D body poses)

## ROS Topics 
- /humans/poses/body/2D
- /humans/poses/body/3D
- /humans/poses/body/3D/markers
- /humans/poses/gaze/3D
- /humans/poses/gaze/3D/markers
- /humans/visual_attention
- /humans/memory


## [Human Perception Package](./human_perception)
- [x] rt-gene integration
- [x] openpose integration
- [x] 2D to 3D body poses, using depth
- [x] fusion of head and body

## [Human Visual Attention Package](./human_visual_attention)
- [x] dynamic world
- [x] human head in bullet
- [x] human body in bullet
- [x] PR2 in bullet
- [x] Pepper in bullet
- [ ] change sigma regarding head speed
- [x] compute center of attention first
- [x] get element of attention
- [x] compute intensity based on distance with center
- [x] define an attention structure for each element
- [ ] store time of attention
- [ ] clean cpp code with headers
- [ ] publish human attention cumulated (WIP)
- [ ] publish discrete human attention cumulated (WIP)
- [ ] publish human memory (WIP)

## [UWDS Human Clients Package](./uwds_human_clients)
- [x] gaze to uwds
- [x] body to uwds
- [ ] PR2 to uwds
- [ ] Pepper to uwds
- [ ] create a new World for each Human

## [Human Experiments Package](human_experiments)
- [x] full gaze provider from PR2 Kinect
- [x] full gaze provider from webcam
- [ ] full gaze provider from Pepper
- [x] full body provider from PR2 Kinect
- [ ] full body provider from Pepper
- [ ] full human provider from PR2 Kinect (not tested recently)
- [ ] full human provider from Pepper
- [ ] full visual attention (WIP)

## [Human Viz Package](./human_viz)
- [x] viz of body
- [x] viz of head
- [x] viz of human (body+head)
- [ ] refactoring for common code
- [X] viz of human attention
- [ ] viz of human memory
