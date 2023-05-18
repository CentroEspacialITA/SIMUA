# 🔴 SSL Related Errors 🔴

The SSL certificate error message you are receiving is likely due to the incorrect system time on your Ubuntu 22.04 server. This can happen if the system clock is not synchronized with the correct time or if the system time zone is set incorrectly.

To resolve this issue, you can try the following steps:

1. Set the correct time zone for your server by running the following command: ```sudo timedatectl set-timezone [timezone]```

Replace **[timezone]** with the name of your time zone (e.g. "America/New_York" or "Europe/London").

2. Ensure that your system clock is synchronized with a reliable time source by running the following command: ```sudo timedatectl set-ntp true```

This will enable network time synchronization, which will ensure that your system clock stays accurate.

3. Check the current time and date on your server by running the following command: ```date```

If the date and time are still incorrect, you can manually set them by running the following command: ```sudo date -s "YYYY-MM-DD HH:MM:SS"```

After you have completed these steps, try accessing the website again and see if the SSL certificate error has been resolved. If you continue to experience issues, you may need to check if the SSL certificate itself is valid and not expired or revoked.

---

# 🔴 ROS2 Installation on Ubuntu 22.04 Server 🔴

* Official Doc: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

* Set Locale:

```
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

* Setup Sources:

```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now add the ROS 2 GPG key with apt.

```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Then add the repository to your sources list.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys.

```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

* Install ROS2 Packages:

Update your apt repository caches after setting up the repositories.

```
sudo apt update
```

ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.

```
sudo apt upgrade
```

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

```
sudo apt install ros-humble-desktop-full
```

ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.

```
sudo apt install ros-humble-ros-base
```

Development tools: Compilers and other tools to build ROS packages

```
sudo apt install ros-dev-tools
```

* Sourcing the setup script

```
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash
```

---

# 🔴 ROS2 Multiple Machines Tutorial 🔴

## Network configuration for ROS2 multi-machines
Before starting, obviously make sure you have installed ROS2 on each machine, preferably the same distribution.

Then, make sure you don’t have a firewall blocking your communications on the network. If you have a firewall, allow UDP multicasting, or disable the firewall at least during your first tests.

Connect all your machines in the same network. This is very important, otherwise they simply won’t be able to find each other.

Now, you can easily check if the machines can reach out to each other.

First, get the IP address of each machine inside the network by running ```hostname -I```

Example:

    Machine 1: hostname -I
    returns 192.168.43.138 172.17.0.1
    Machine 2: hostname -I
    returns 192.168.43.56

You may have multiple IP addresses on each machine, depending on what you’ve previously configured (ex: on Machine 1 I have Docker, this is why you see 172.17.0.1). Just find the IP addresses that are on the same network, here those who start with 192.168.43.xx.

If you can ping the machines from each other then the network configuration is done.

* Machine 1:
```
$ ping 192.168.43.56
PING 192.168.43.56 (192.168.43.56) 56(84) bytes of data.
64 bytes from 192.168.43.56: icmp_seq=1 ttl=64 time=128 ms
64 bytes from 192.168.43.56: icmp_seq=2 ttl=64 time=136 ms
```
* Machine 2:
```
$ ping 192.168.43.138
PING 192.168.43.138 (192.168.43.138) 56(84) bytes of data.
64 bytes from 192.168.43.138: icmp_seq=1 ttl=64 time=8.75 ms
64 bytes from 192.168.43.138: icmp_seq=2 ttl=64 time=132 ms
```

## Run ROS2 on 2 machines
Now it’s very simple. All you have to do is to start some nodes in Machine 1, some other nodes in Machine 2, and they will all be able to communicate through topics, services and actions. Just like they were all in the same machine.

* Machine 1:
```
$ source /opt/ros/your_ros2_distribution/setup.bash # You can put that line into your ~/.bashrc
$ ros2 run demo_nodes_cpp talker
```

* Machine 2:
```
$ source /opt/ros/your_ros2_distribution/setup.bash # You can put that line into your ~/.bashrc 
$ ros2 run demo_nodes_cpp listener
```

And you should see logs on both machines!

If you want to communicate with a third/fourth/… machine, simply follow the network configuration steps again, and you’ll be all set

## Use ROS_DOMAIN_ID to run multiple (separate) ROS2 applications on the same network


So, after you’ve configured the machines to be in the same network, they are all part of the same ROS2 application. This can be a problem: what if you want to run 2 different ROS2 applications on the same network and on multiple machines? Here you might want to completely separate the applications from each other.

Well, that’s possible, you just need to set one environment variable before you start your nodes.

Before you start any node in one session (= one terminal), you need to export a new environment variable, named ROS_DOMAIN_ID, using a number for the value (preferably a low number, between 1 and 232). Then, only the nodes started in sessions with the same ROS_DOMAIN_ID will be able to communicate with each other.

Example:
* Machine 1:
```
$ export ROS_DOMAIN_ID=5
$ source /opt/ros/your_ros2_distribution/setup.bash
$ ros2 run demo_nodes_cpp talker
```
* Machine 2:
```
$ export ROS_DOMAIN_ID=5 
$ source /opt/ros/your_ros2_distribution/setup.bash 
$ ros2 run demo_nodes_cpp listener
```
Try to set a different ROS_DOMAIN_ID (or don’t set one at all) on Machine 2 – session B, and you’ll see that the communication won’t work.