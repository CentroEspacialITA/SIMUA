# RPLidar [![Documentation](https://readthedocs.org/projects/rplidar/badge/?version=latest)](http://rplidar.readthedocs.org/en/latest/?badge=latest) [![PyPI](https://img.shields.io/pypi/v/rplidar.svg)](https://pypi.python.org/pypi/rplidar) [![MIT License](https://img.shields.io/github/license/mashape/apistatus.svg)](https://github.com/SkRobo/rplidar/blob/master/LICENSE)

Simple and lightweight Python module for working with RPLidar rangefinder scanners.

This module aims to implement communication protocol with RPLidar rangefinder
scaners. It's Python 2 and 3 compatible, but was mainly tested using Python 3.

For protocol specifications please refer to the slamtec
[document](http://www.slamtec.com/download/lidar/documents/en-us/rplidar_interface_protocol_en.pdf).

## Installing

You can install rplidar using `pip`:

```sh
$ pip install rplidar-roboticia
```

Or for Python 3:
```sh
$ sudo pip3 install rplidar-roboticia
```

## Documentation

View the latest rplidar documentation at http://rplidar.rtfd.org/.

## Usage example

Simple example:
```Python
from rplidar import RPLidar
lidar = RPLidar('/dev/ttyUSB0') # to confirm the device, please type in terminal: ls /dev/ -l

info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

for i, scan in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(scan)))
    if i > 10:
        break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()
```

In addition to it you can view example applications inside
[examples](https://github.com/SkRobo/rplidar/tree/master/examples>) directory.


# IMPORTANT

If you get an error like this:

```bash
raise RPLidarException('Failed to connect to the sensor '
rplidar.RPLidarException: Failed to connect to the sensor due to: [Errno 13] could not open port /dev/ttyUSB0: [Errno 13] Permission denied: '/dev/ttyUSB0'
```

when running any example, please follow these steps:

1. Add User to the dialout Group:

On many Linux systems, access to serial ports (like /dev/ttyUSB0) is restricted to users in certain groups. The dialout group is often used for this purpose. You can add your user to the dialout group using the following command:

```bash
sudo usermod -aG dialout YOUR_USERNAME_HERE
```

After running this command, you'll need to log out and log back in for the group changes to take effect.

2. Grant Access to the Device:

You can also grant read and write access to the specific device file (/dev/ttyUSB0) to the user. This can be done by adjusting the device's permissions:

```bash
sudo chmod a+rw /dev/ttyUSB0
```