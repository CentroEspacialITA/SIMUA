# 🔴 ROS 2 (Humble) 🔴

**Obs: you can run ```auto-gen.sh``` (linux) for an auto generated environment using:**

```
sudo bash auto-gen.sh
```

**, otherwise follow the instructions below**

--- 
## ✅ How to Create a ROS Environment (Sandbox)
The main tutorial can be find here: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html


### Tasks:

#### 🔺 1. Source ROS 2 environment 🔺

Your main ROS 2 installation will be your underlay for this tutorial. (Keep in mind that an underlay does not necessarily have to be the main ROS 2 installation.)

```source /opt/ros/humble/setup.bash```

#### 🔺 2. Create a new directory 🔺

Best practice is to create a new directory for every new workspace. The name doesn’t matter, but it is helpful to have it indicate the purpose of the workspace. Let’s choose the directory name ```ros2_ws```, for “development workspace”:

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Another best practice is to put any packages in your workspace into the src directory. The above code creates a ```src``` directory inside ```ros2_ws``` and then navigates into it.

#### 🔺 3. Clone a sample repo 🔺

Ensure you’re still in the ```ros2_ws/src``` directory before you clone.

In the rest of the beginner developer tutorials, you will create your own packages, but for now you will practice putting a workspace together using existing packages.

If you went through the Beginner: CLI Tools tutorials, you’ll be familiar with ```turtlesim```, one of the packages in ros_tutorials.

A repo can have multiple branches. You need to check out the one that targets your installed ROS 2 distro. When you clone this repo, add the ```-b``` argument followed by that branch.

In the ```ros2_ws/src``` directory, run the following command:

```git clone https://github.com/ros/ros_tutorials.git -b humble-devel```

Now ```ros_tutorials``` is cloned in your workspace. The ```ros_tutorials``` repository contains the ```turtlesim``` package, which we’ll use in the rest of this tutorial. The other packages in this repository are not built because they contain a ```COLCON_IGNORE``` file.

So far you have populated your workspace with a sample package, but it isn’t a fully-functional workspace yet. You need to resolve the dependencies first and then build the workspace.

#### 🔺 4. Resolve dependencies 🔺

Before building the workspace, you need to resolve the package dependencies. You may have all the dependencies already, but best practice is to check for dependencies every time you clone. You wouldn’t want a build to fail after a long wait only to realize that you have missing dependencies.

From the root of your workspace ```(ros2_ws)```, run the following command:

```
# cd if you're still in the ``src`` directory with the ``ros_tutorials`` clone
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```

If you already have all your dependencies, the console will return: ```#All required rosdeps installed successfully```

Packages declare their dependencies in the package.xml file (you will learn more about packages in the next tutorial). This command walks through those declarations and installs the ones that are missing.

#### 🔺 5. Build the workspace with colcon 🔺

From the root of your workspace ```(ros2_ws)```, you can now build your packages using the command: ```colcon build```

The console will return the following message:
```
Starting >>> turtlesim
Finished <<< turtlesim [5.49s]

Summary: 1 package finished [5.58s]
```

Once the build is finished, enter ls in the workspace root ```(~/ros2_ws)``` and you will see that colcon has created new directories:

```build  install  log  src```

The ```install``` directory is where your workspace’s setup files are, which you can use to source your overlay.

---
## ✅ Writing a simple publisher and subscriber (Python)

Goal: Create and run a publisher and subscriber node using Python.

### Background

In this tutorial, you will create nodes that pass information in the form of string messages to each other over a topic. The example used here is a simple “talker” and “listener” system; one node publishes data and the other subscribes to the topic so it can receive that data.

### Prerequisites

In previous tutorials, you learned how to create a workspace and create a package. A basic understanding of Python is recommended, but not entirely necessary.

### Tasks

#### 🔺 1. Create a package 🔺

Open a new terminal and source your ROS 2 installation so that ros2 commands will work.

Navigate into the ```ros2_ws``` directory created in a previous tutorial.

Recall that packages should be created in the ```src``` directory, not the root of the workspace. So, navigate into ```ros2_ws/src```, and run the package creation command:

```ros2 pkg create --build-type ament_python py_pubsub```

Your terminal will return a message verifying the creation of your package ```py_pubsub``` and all its necessary files and folders.

#### 🔺 2. Write the publisher node 🔺

Navigate into ```ros2_ws/src/py_pubsub/py_pubsub```. Recall that this directory is a Python package with the same name as the ROS 2 package it’s nested in.

Download the example talker code by entering the following command:

```
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

Now there will be a new file named ```publisher_member_function.py``` adjacent to ```__init__.py```.

Open the file using your preferred text editor.

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

> ---

> * Adding dependencies:

> Navigate one level back to the ```ros2_ws/src/py_pubsub``` directory, where the ```setup.py```, ```setup.cfg```, and ```package.xml``` files have been created for you.

> Open ```package.xml``` with your text editor. As mentioned in the previous tutorial, make sure to fill in the ```<description>```, ```<maintainer>``` and ```<license>``` tags:

>
```
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

> After the lines above, add the following dependencies corresponding to your node’s import statements:

>
```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

> This declares the package needs ```rclpy``` and ```std_msgs``` when its code is executed. Make sure to save the file.

> * Adding an entry point:

> Open the ```setup.py file```. Again, match the maintainer, maintainer_email, description and license fields to your package.xml:

>
```
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```

> Add the following line within the ```console_scripts``` brackets of the ```entry_points``` field:

>
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

> ---

> * Checking setup.cfg:

> The contents of the ```setup.cfg``` file should be correctly populated automatically, like so:

>
```
[develop]
script_dir=$base/lib/py_pubsub
[install]
install_scripts=$base/lib/py_pubsub
```

> This is simply telling setuptools to put your executables in ```lib```, because ros2 ```run``` will look for them there. You could build your package now, source the local setup files, and run it, but let’s create the subscriber node first so you can see the full system at work.

#### 🔺 3. Write the subscriber node 🔺

Return to ```ros2_ws/src/py_pubsub/py_pubsub``` to create the next node. Enter the following code in your terminal:

```
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

Now the directory should have these files:
```
__init__.py  publisher_member_function.py  subscriber_member_function.py
```

Open the ```subscriber_member_function.py``` with your text editor.

```
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

The subscriber node’s code is nearly identical to the publisher’s. The constructor creates a subscriber with the same arguments as the publisher. Recall from the topics tutorial that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.

> ---

> * Adding an entry point:

> Reopen setup.py and add the entry point for the subscriber node below the publisher’s entry point. The entry_points field should now look like this:

```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

> Make sure to save the file, and then your pub/sub system should be ready.

#### 🔺 4. Build and run 🔺

You likely already have the ```rclpy``` and ```std_msgs``` packages installed as part of your ROS 2 system. It’s good practice to run ```rosdep``` in the root of your workspace ```(ros2_ws)``` to check for missing dependencies before building:

```
rosdep install -i --from-path src --rosdistro humble -y
```

Still in the root of your workspace, ros2_ws, build your new package:

```
colcon build --packages-select py_pubsub
```

Open a new terminal, navigate to ```ros2_ws```, and source the setup files:

```
source install/setup.bash
```

Now run the talker node:

```
ros2 run py_pubsub talker
```

The terminal should start publishing info messages every 0.5 seconds, like so:

```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
[INFO] [minimal_publisher]: Publishing: "Hello World: 3"
[INFO] [minimal_publisher]: Publishing: "Hello World: 4"
...
```

Open another terminal, source the setup files from inside ```ros2_ws``` again, and then start the listener node:

```
ros2 run py_pubsub listener
```

The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

```
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
...
```
