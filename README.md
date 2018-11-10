# Nortek DVL1000 ROS Interface Layer
by your friends at ARVP from University of Alberta

Connects to DVL over Ethernet (port:9004) and publishes DVL data + status.
Velocities are output in m/s (or NaN if invalid).
Custom messages are used for both publishers.

**Launching the node:** `roslaunch nortek_dvl dvl.launch`

### Parameters

* **address** IP address / host name of DVL
* **port** port number for TCP connection
* **dvl_topic** dvl data pub topic name
* **dvl_status_topic** dvl status pub topic namei
* **dvl_rotation** rotate dvl data if the DVL coods do not match the robot coods (in radians)


*please note that both publishers are in the nodes private namespace*

### DVL configuation

check the [DVLConfig.txt](DVLConfig.txt) file for our DVL configuration.
