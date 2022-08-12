# Demo Instructions



## Hardware

1. Jetson Nano 1 as the Robot Terminal  (waggle-ized as an "worker agent")

2. Jetson Nano 2 as the Master/Controller/Local Scheduler (waggle-ized as "core")

3. Opentrons OT2

4. (?) Networking equipment



# Preparation

1. The Robot Terminal (Nano) should be connected to the same network as the OT2 Robot, with a configuration/access level that will allow the underlying HTTP requests from the robot terminal to the robot. 

2. With another computer that has the Opentrons App pre-installed, connect to the OT2 by USB (via an internal USB-to-Ethernet adapter, link-only mode). The versions of the app and OT2 firmware must be compatible, so, if not yet, update both to their respective latest.

3. From within the Opentrons App, note the IP address of the robot on the wireless network.

NOTE: The Opentrons App does not work on the Jetson Nano (arm64 architecture), which is why step 2 above requires a second computer just to note the ip address. For the same reason, there is no way to see the continually changing IP address if the Nano was instead networked to the OT2 via USB/Ethernet.

4. Network Requirements of Waggle devices



## Setup



### Download Docker images

The <u>Robot Terminal</u> ("worker agent") should be provisioned with its docker image:

`docker pull kwelbeck/demo-ot2`



And for the <u>Master</u> ("core") :

`docker pull kwelbeck/demo-master`	



### Run the docker images with required environment variables

<u>Robot Terminal:</u>

`docker run -it -e NAME=ot2_1 -p 80:80 kwelbeck/demo-ot2 `

- Port 80 of the container is mapped to port 80 of the host Nano to facilitate the http communication with the OT2
- the `NAME=ot2_1` specifies a name space used in the underlying ROS2 implementation



<u>Master:</u>

`docker run -it  -e robot_ip=xxx.xxx.xxx.xxx kwelbeck/demo-master`

- where `xxx.xxx.xxx.xxx` is the ip address of the robot from **Preparation step 3**



Running both containers on their respective Nanos, parses and executes a "WorkFlow" document that is (for now) pre-loaded on the demo-master container. The "WorkFlow" contains the experiment description that, from the Master/Controller, translates into respective responsibilities for the robots and robot terminals  (a.k.a waggle devices) that make up the "WorkCell".

**As such the OT2 should run the pre-configured protocol that was passed to it as part of the "WorkFlow"**



## Demonstration Items

[To Do: List of capabilities/integrations to be demonstrated]