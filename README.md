[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive) ![completion](https://img.shields.io/badge/completion%20state-100%25-blue.svg?style=plastic)<br><br><br>
![level5-engineers](https://avatars1.githubusercontent.com/u/31551095?v=4&s=100)
### level5-engineers
<b>CarND Capstone Project: System Integration</b><br>
<em>It’s not about the pieces but how they work together.</em>
<br>

#### Team

* James Dunn, lead
* Oleg Leizerov
* Aman Agarwal
* Rajesh Bhatia
* Yousof Ebneddin

### Objective
Create code to drive a vehicle in both a Unity-based simulator and a real-world Lincoln MKZ around a closed-circuit test-track. This repository contains all ROS nodes to implement the core functionality of an autonomous vehicle system.

### Results
From the Udacity review: "Excellent work here! The car drove very smoothly around the waypoints, and made a full stop for the red light. Well done!" 
- Video from the dash camera onboard the test vehicle: <a href="https://vimeo.com/241961307" target="_blank">here</a> (Drive-by-wire is engaged at 2s and disengaged at 38s.) 
- A map of the test run can be found ![here](https://github.com/level5-engineers/system-integration/wiki/Site-Test#second-review-final)
- Log file, ROS bag, and feedback: <a href="https://udacity-selfdrivingcar.s3.amazonaws.com/reviews/834316_1510097183.zip" target="_blank">here</a>

### Implementation Notes

The diagram below illustrates the system architecture. The autonomous vehicle controller is composed of three major units: perception, planning, and control.

![System Architecture](https://raw.githubusercontent.com/level5-engineers/assets/master/images/SystemArchitecture.png)
Legend: the letters a-k indicate published ROS topics

```
  a: /camera/image_raw
  b: /current_pose
  c: /current_velocity
  d: /vehicle/dbw_enabled
  e: /traffic_waypoint
  f: /base_waypoints
  g: /final_waypoints
  h: /twist_cmd
  i: /vehicle/throttle_cmd
  j: /vehicle/brake_cmd
  k: /vehicle/steering_cmd
```

### Perception
We employ the MobileNet architecture to efficiently detect / classify traffic lights. We applied transfer learning to further train two convolutional neural networks for the different modes of operation:
- simulator mode: classifies whole images as either `red` or `none`. The model was trained with several datasets using the Tensorflow Image Retraining Example (tutorial: https://goo.gl/HgmiVo, code: https://goo.gl/KdVcMi). 
- test-site mode: we employ the "SSD: Single Shot MultiBox Detection" framework to locate a bounding box around a traffic light. We fine-tuned the pre-trained `ssd_mobilenet_v1_coco` model using the Tensorflow Object Detection API. The training dataset includes camera images from training, reference, and review rosbags.
- [more...](#additional-specifications)

### Planning
The waypoint updater node publishes a queue of `n` waypoints ahead of the vehicle position, each with a target velocity. For the simulator, `n=100` is sufficient. For the site (the real-world test track), we reduce to `n=20`. We dequeue traversed waypoints and enqueue new points, preserving and reusing those in the middle. When a light-state changes, the entire queue is updated. The vehicle stops at the final base waypoint. [more...](#additional-specifications)

### Control
The drive-by-wire node adjusts throttle and brakes according to the velocity targets published by the waypoint follower (which is informed by the waypoint updater node). If the list of waypoints contains a series of descending velocity targets, the PID velocity controller (in the twist controller component of DBW) will attempt to match the target velocity. [more...](#additional-specifications)

### Operation
There are three modes in which the controller operates: 
- site: When at the test site, this mode is launched. This mode can be run simultaneously with a rosbag to test the traffic light classifier. (See below)
- sitesim: emulates the test site in the simulator at the first traffic light.
- styx: When using the term3 simulator, this mode is launched. The simulator communicates through server.py and bridge.py

These modes are started by roslaunch. For example, to run the styx (simulator) version we run:

`roslaunch launch/styx.launch`

### Additional Specifications

[Traffic light image classification](https://github.com/level5-engineers/system-integration/wiki/Traffic-Light-Image-Classification)<br>
[Waypoint updater](https://github.com/level5-engineers/system-integration/wiki/Waypoint-Updater)<br>
[Drive-by-wire](https://github.com/level5-engineers/system-integration/wiki/Drive-By-Wire)<br>

### References

[Traffic Light Detection and Classification](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62)<br>
[SSD: Single Shot MultiBox Detection](https://arxiv.org/pdf/1512.02325.pdf)<br>
[Machine learning](http://ftp.cs.wisc.edu/machine-learning/shavlik-group/torrey.handbook09.pdf)<br>
[MobileNets](https://arxiv.org/abs/1704.04861)<br>
[Transfer learning](http://ruder.io/transfer-learning/index.html)<br>
[Pure Pursuit Algorithm](http://ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)<br>
[Quaternion mathematics](https://web.archive.org/web/20120417090529/http://www.itu.dk/people/erikdam/DOWNLOAD/98-5.pdf)<br>
[Quaternions online visualization](http://quaternions.online/)<br>
[PID control](https://udacity-reviews-uploads.s3.amazonaws.com/_attachments/41330/1493863065/pid_control_document.pdf)<br>

---

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project wiki [here](https://github.com/level5-engineers/system-integration/wiki).

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/level5-engineers/system-integration.git
```

2. Install python dependencies
```bash
cd system-integration
pip install -r requirements.txt
```

3. Make the controller
```bash
cd ros
catkin_make
```

4. In a new terminal window, start roscore
```bash
roscore
```

5. Start the simulator, select screen resolution 800x600, click SELECT, uncheck the Manual checkbox. Ideally, run the simulator in the host environment (outside of the virtual machine).

6. In a new terminal window, start the controller
```bash
cd system-integration/ros
source devel/setup.sh
roslaunch launch/styx.launch
```

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd system-integration/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
