ros_node_transformation
---

The package contains a node that can flexibly create a list of devices at runtime based on a configuration file. Each device can dynamically load a different publisher at runtime and change the type of data that pushes into the ROS network. It's an example application that showcases how a generic code along with a configuration file and the power of plugins can be used to create what would otherwise be multiple instances of repetitive and hardcoded pieces of code. There is an accompanying [blog post](http://nlamprian.me/blog/software/ros/2019/04/20/ros-node-transformation/) that describes this concept.

![robot-rviz](https://github.com/nlamprian/nlamprian.github.io/blob/master/assets/img/blog/2019-04-20-ros-node-transformation/robot-rviz.gif)

Test
---

Install socat

```bash
sudo apt install socat
```

Create a directory for the serial ports

```bash
mkdir ~/dev
```

Create the serial ports

```bash
socat -d -d pty,raw,echo=0,link=/home/`whoami`/dev/ttyIMU pty,raw,echo=0,link=/home/`whoami`/dev/ttyIMU0 &
socat -d -d pty,raw,echo=0,link=/home/`whoami`/dev/ttySonarFront pty,raw,echo=0,link=/home/`whoami`/dev/ttySonarFront0 &
socat -d -d pty,raw,echo=0,link=/home/`whoami`/dev/ttySonarRear pty,raw,echo=0,link=/home/`whoami`/dev/ttySonarRear0 &
```

Write some data to the ports

```bash
function get_random {
  awk -v n=$1 -v scale=$2 -v seed="$RANDOM" 'BEGIN { srand(seed); for (i=0; i<n; ++i) { printf("%.4f", scale*rand()); if (i<n-1) printf(","); else printf("\n"); } }'
}

while true; do
  get_random 9 1 > ~/dev/ttyIMU0;
  get_random 1 10 > ~/dev/ttySonarFront0;
  get_random 1 10 > ~/dev/ttySonarRear0;
  sleep 0.02;  # 50 Hz
done
```

Install dependencies

```bash
sudo apt install ros-${ROS_DISTRO}-serial
# cd to <catkin_ws>/src
git clone https://github.com/nlamprian/ros_node_configuration.git
```

Download the package, compile and source the workspace, and finally start the launch file

```bash
roslaunch ros_node_transformation bringup.launch
```

The devices will publish data to a topic and diagnostic statuses.

To visualize the data in rviz, start with

```bash
roslaunch ros_node_transformation bringup.launch start_rviz:=true
```
