# Oculus sonar ROS1

ROS1 node for the Blueprint Subsea Oculus sonar.

## Getting started

### Installation (with internet connection)

**The oculus_ros node is merely a wrapper and depends on an external
oculus_driver library which does most of the work. This library will be
automatically downloaded during the colcon build process. If you don't have an
internet connection available, see the instruction further below.**

Go to your catkin workspace "src" folder:
```
cd <your catkin workspace>/src
```

Clone with repository.
```
git clone https://github.com/ENSTABretagneRobotics/oculus_ros.git
```

Build the oculus_sonar ROS node:
```
cd .. && catkin_make
```

Check you can start the node:
```
rosrun oculus_sonar oculus_sonar_node
```

All set !



### Installation (without an internet connection)

#### Install the oculus_driver library

If you don't have an internet connection available on the system on which you
want to use this node, your have to install the
[oculus_driver](https://github.com/ENSTABretagneRobotics/oculus_driver.git)
library beforehand.

Clone or copy the oculus_driver library :
```
git clone https://github.com/ENSTABretagneRobotics/oculus_driver.git
```

This library follows a standard CMake compilation procedure. cd into the repo
and create a build directory :

```
cd oculus_driver && mkdir build && cd build
```

Generate your build system with CMake, compile and install :
```
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=<your install location> ..
make -j4 install
```

Make sure the CMAKE_PREFIX_PATH environment variable contains your install
location :
```
echo $CMAKE_PREFIX_PATH
```

If not, put this at the end your $HOME/.bashrc file:
```
export CMAKE_PREFIX_PATH=<your install location>:$CMAKE_PREFIX_PATH
```

Now follow the oculus_ros node installation procedure normally. If the
oculus_driver is properly installed **at a location included in the
CMAKE_PREFIX_PATH environment variable**. The node should compile properly.



## How it works (in brief)

#### Network configuration

The Oculus sonar connects to the system through the network. The sonar itself
has a fixed IP address which may or may not be indicated on the box. To avoid
bricking of the sonar by "lack of post-it", the sonar makes himself known on the
network by broadcasting its own IP address. The IP address is therefore always
retrievable, regardless of your system network configuration.

The oculus_sonar library should always detect the IP of a plugged in Oculus
sonar and will print it to stdout. If you do not have access to stdout, a tool
such as wireshark will display the packets broadcasted by the sonar along with
its IP address. The ROS node also publish the sonar configuration in the topic
"/oculus_sonar/status", although it might not be in a user readable format
(encoded in a uint32_t, for now).

You should make sure your own system configuration match the configuration of
the sonar (i.e. your system and the sonar must be on the same subnet). If you are
unable to change your own configuration, the IP of the sonar can be changed with
the help of the Oculus ViewPoint software (made for Windows, but works with wine
on Ubuntu).

#### General operation

**Always make sure the sonar is underwater before powering it !**

In normal operation the sonar will continuously send ping. Various ping
parameters can be changed (ping signal frequency, ping rate...). The sonar will
retain its current configuration between power cycles. This makes very important
to **not** power up the sonar before putting it underwater. It may send pings
even if the ROS node is not launched (it should not break right away but will
heat up very fast).

If the ROS node is launched, it will stop the ping emission if there are no
subscribers on the /oculus_sonar/ping topic.


The ping parameters can be changed using the rqt_reconfigure graphical tool.
```
rosrun rqt_reconfigure rqt_reconfigure
```
The sonar might take a lot of time to acknowledge a parameter change (especially
parameters related to sound velocity and salinity).


## Troubleshooting

Most of the issue you ~~might~~ will encounter are related to network setup.
Check the Network configuration section.

For other problems, feel free to contact the maintainer at
pierre.narvor@ensta-bretagne.fr.

