# VPA DB19 Interface

This reporepository is a package that drives the DB19 hardware. The code is a combination of 
+ Relavant and minimun selection from Duckietown offical interface packages, [dt-duckiebot-interface](https://github.com/duckietown/dt-duckiebot-interface)
+ Simple PI controller that control wheel spinning speed based on feedback of the encoder

> [!NOTE]
> This package is not in a docker container like the original Duckietown library as we find it more friendly to beginners of the education purpose. This may not be the best practice!

## Environments and Setup
### Operating System and ROS
The package operate only with [ROS Noetic](http://wiki.ros.org/noetic/Installation), which requires a ubuntu 20.04 system. Now the SNAM lab installed [Ubuntu-Mate 20.04](https://releases.ubuntu-mate.org/archived/).
Please be aware that the 20.04 image from the website is not working properly as introduced in this [discussion](https://ubuntu-mate.community/t/error-when-installing-ubuntu-mate-20-04-2-lts-on-raspberry-pi-3b/23893). A practice offered here is to first install 18.04 and upgrade.

### Possible requried packages
Generally the rosdep command should solve the dependencies, but in case of something does not work, the following packages are required
+ python-is-python3
+ python3-smbus

### Camera
To enable raspi camera, you may need install [raspi-config](https://www.raspberrypi.com/documentation/computers/configuration.html) (already installed if started from Ubuntu-mate 18.04) and enable camera interface.

## ROS Topics and Dynamic Reconfiguration
### Topics
#### Subscribe
+ **cmd_vel**: relative name, standard format as in [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html). Please be noted that the wheel is programmed from not going backwards, but it does brake.
#### Publish
+ **~tick**: customized msgs, as shown in WheelEncoderStamed.msg. It contains the spinning speed (omega,rad/s), 
+ **Wheels_Cmd**: cistomized msgs, as shown in WheelsCmd.msg. It contains the throttle towards both wheels (-1 to 1)

### Dynamic Reconfigure
It is possible to tune the P and I parameters of the wheel spining controller with a dynamic reconfigure interface.