# 
Ros community offers really nice "plug and play" packages to control different modules. Unfortunately, this is not such a module, basically becuase most of the time robotic projects requires more complicated control structures which seamlessly bridge low level controller needs to high level motion planning and tasks.  is a package intending to address this problem.
Some of possible usages of this package are ;
* Controlling your new bizarre robot with presented closed loop controller (FeedbackControllers, ModelPredictiveControllers etc.). To do so you your simulation or real robot should publish the states of the robot through ROS topics and should listen a ROS topic for a control input.
* Controlling your standard robot,which is controlled by open source ROS controllers, in a open-loop manner. If you are working on path planing on industrial robots, this package will offer you orgainzed way of controlling your robot.
* Controlling custom assebled robot. Most of the case in the robotic research, you will end up with a robot with some random mobile base and a joint position/veloctiy control arm. This package ease the integration of process for such robotic configurations.
* Controlling multiple robots in a centralized manner. You can create a controller frame which contains controllers of multiple robots with this package. This way, you can centralize information flow in your multi robot system.
* Controlling multiple robots in a decentralize manner. Mobile base producers are generally some twisted jerks who produces robots with its own ROScore to make you suffer (just joking, we love them as well). Therefore, you can also handle control of such system using  (a tutorial will be prepared for this later.)


## Dependencies
* [ros_node_utils](https://github.com/Sadetra/ros_node_utils)

## Installization
Simply clone and add it to your catkin workspace.

## Tutorials
Later, an example system will be available here. 


## System Requirements
ros_node_utils package requires following system specifications; 
* [Ubuntu](https://www.ubuntu.com/) 16.04
* [GCC](https://gcc.gnu.org/) 5.4.0 
* or [Python 2.7]
* [Ros](http://www.ros.org/)-kinetic
* Eigen 3.3 (Ubuntu 16.04 libeigen-dev bulunduruyor)
* Boost 1.58.0
* [catkin_tools](http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) 

