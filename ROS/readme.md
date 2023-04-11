# SSL Related Errors:

The SSL certificate error message you are receiving is likely due to the incorrect system time on your Ubuntu 22.04 server. This can happen if the system clock is not synchronized with the correct time or if the system time zone is set incorrectly.

To resolve this issue, you can try the following steps:

1. Set the correct time zone for your server by running the following command: ```sudo timedatectl set-timezone [timezone]```

Replace **[timezone]** with the name of your time zone (e.g. "America/New_York" or "Europe/London").

2. Ensure that your system clock is synchronized with a reliable time source by running the following command: ```sudo timedatectl set-ntp true```

This will enable network time synchronization, which will ensure that your system clock stays accurate.

3. Check the current time and date on your server by running the following command: ```date```

If the date and time are still incorrect, you can manually set them by running the following command: ```sudo date -s "YYYY-MM-DD HH:MM:SS"```

After you have completed these steps, try accessing the website again and see if the SSL certificate error has been resolved. If you continue to experience issues, you may need to check if the SSL certificate itself is valid and not expired or revoked.

# ROS2 Installation on Ubuntu 22.04 Server

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
