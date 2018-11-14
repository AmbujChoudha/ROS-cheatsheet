# ROS-cheatsheet
A ROS cheat sheet for learning and quick reference.

# Table of contents
1. [Environment variables](#env)
2. [ROS commands](#ros-cmd)
    - [roscore](#roscore)
    - [rosrun](#rosrun)
    - [rosnode](#rosnode)
    - [rostopic](#rostopic)
    - [rosmsg](#rosmsg)
    - [rosservice](#rosservice)
    - [rossrv](#rossrv)
3. [ROS tools](#tools)
    - [rqt_graph](#rqt-graph)
    - [rqt_plot](#rqt-plot)

<hr>

1. <a name="env">**Environment varibles**</a>
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
2. <a name="ros-cmd">**ROS commands**</a>
    - <a name="roscore">`roscore`</a> : run ros master (provides name service for ROS) + rosout (stdout/stderr) + parameter server
    - <a name="rosrun">`rosrun <ros-package> <ros-name>`</a> ([options](http://wiki.ros.org/Remapping%20Arguments)): run a ros node from some package
    - <a name="rosnode">***rosnode***</a>
        - `rosnode list`: list all running/uncleaned nodes
        - `rosnode info <node-name>`: print info about the node
        - `rosnode cleanup`: clean up 'trash' nodes
        - `rosnode ping <node-name>`: test is the node up
    - <a name="rostopic">***rostopic***</a>
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
                $ rostopic pub /turtle1/cmd_vel geometry_msg/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
                ```
                - option `-r 1` : publish the message recursively at 1 Hz
        - `rostopic hz <topic-name>` : show how fast data is being published to the topic
    - <a name="rosmsg">`rosmsg show <type-name>`</a> : show details of the message type
    - <a name="rosservice">***rosservice***</a>
        - `rosservice list` : print information about active services
        - `rosservice call <service-name> <args>` : call the service with the provided args
        - `rosservice type <service-name>` : print service type
        - `rosservice find <type-name>` : find services by service type
        - `rosservice uri <service-name>` : print service ROSRPC uri
    - <a name=rossrv>***rossrv***</a>
        - rossrv is a command-line tool for displaying information about ROS Service types
        - `rossrv show` : show service description
        - `rossrv info` : alias for rossrv show
        - `rossrv list` : list all services
        - `rossrv md5` : display service md5sum
        - `rossrv package` : list services in a package
        - `rossrv packages` : list packages that contain services
3. <a name="tools">**ROS tools**</a>
    - <a name="rqt-graph">***rqt_graph***</a>
        ```
        $ rosrun rqt_graph rqt_graph
        ```
        ![alt text](/images/rqt_graph.png)
    - <a name="rqt-plot">***rqt_plot***</a>
        ```
        $ rosrun rqt_plot rqt_plot
        ```
        ![alt text](/images/rqt_plot.png)