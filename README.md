# ROS-cheatsheet
A ROS cheat sheet for learning and quick reference.

1. **Environment varibles**
    - `printenv | grep ROS`: print ROS environment varibles
    - `echo ROS_<varible-name>`: print specific environment varible
    - `export ROS_<varible-name>:=<value>`: set value for ROS environment varible
    - List of environment varibles:
        - ROS_ETC_DIR
        - ROS_ROOT
        - ROS_MASTER_URI *=http://localhost:11311* ***(recommend)***
        - ROS_VERSION
        - ROS_PYTHON_VERSION
        - ROS_PACKAGE_PATH
        - ROSLISP_PACKAGE_DIRECTORIES
        - ROS_DISTRO
        - ROS_HOSTNAME *=localhost* ***(recommend)***
2. **ROS commands**
    - `roscore` : run ros master (provides name service for ROS) + rosout (stdout/stderr) + parameter server
    - `rosrun <ros-package> <ros-name>` ([options](http://wiki.ros.org/Remapping%20Arguments)): run a ros node from some package
    - `rosnode list`: list all running/uncleaned nodes
    - `rosnode info <node-name>`: print info about the node
    - `rosnode cleanup`: clean up 'trash' nodes
    - `rosnode ping <node-name>`: test is the node up
    - ***`rostopic`***
        - `rostopic -h`: show help message and exit
        - `rostopic echo <topic-name>`: show the data published on a topic
        - ***`rostopic list`***
            - `rostopic list -h`: show help message and exit
            - `rostopic list -v`: display a verbose list of topics to publish to and to subcribe to and their types
        - `rostopic type <topic-name>`: return the message type of the topic being published
        - `rostopic pub <options> [topic] [msg_type] <options> [args]`: publish data on to a topic
            - Example 1:
                ```
                $ rostopic pub -1 /turtle1/cmd_vel geometry_msg/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
                ```
                - option `-1` (dash-one): cause rostopic to publish only one message then exit
                - option `--` (double-dash): tell the option parser that none of the following arguments is an option. This is required in cases where your arguments have a leading dash -, like negative numbers.
            - Example 2: 
                ```
                rostopic pub /turtle1/cmd_vel geometry_msg/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
                ```
                - option `-r 1` : publish the message recursively at 1 Hz
        - `rostopic hz <topic-name>` : show how fast data is being published to the topic
    - `rosmsg show <type-name>`: show details of the message type
3. **ROS tools**
    - ***rqt_graph***
        ```
        $ rosrun rqt_graph rqt_graph
        ```
        ![alt text](/images/rqt_graph.png)
    - ***rqt_plot***
        ```
        $ rosrun rqt_plot rqt_plot
        ```
        ![alt text](/images/rqt_plot.png)