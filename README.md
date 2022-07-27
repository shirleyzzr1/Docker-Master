# Docker-Master

- Builds the demo Docker container to be run on an independent terminal.
- Interfaces with another docker container that is connected and provisioned to instruct an OT2 robot.
- Provides the interface (demo > action_client node) that instructs the OT2 robot with which yaml protocol_config and robot_config to run.  

## Configuation Instructions/Setup Notes

- The host computer and the robot's terminal must be on the same network, verified by successfull pinging.
- This container will require access to the robot configuration and protocol configuration files sent to the OT2 robot. Hence, sample config files are loaded in during the image build.

## Runtime Instructions

1. Clone the repository
`git clone https://github.com/shirleyzzr1/Docker-Master.git`

2. Change directory into the newly-cloned repository
`cd Docker-Master`

3. Build the docker image (might be required to prepend docker commands with sudo)
`docker build -t master`

4. Launch a container instance of the newly-built image
`docker run -it master bash`  ** ports(?)

5. Run the action_client node
`ros2 run demo action_client`

6. In a new terminal, enter into the running container
`docker exec -it <container-id-or-name> bash`

7. Send run instructions to OT2 by specifying the robot config path (rp_path) and the protocol configuration path (pc_path)
`ros2 service call /execute_job demo_interfaces/srv/ExecuteJob "{rc_path: '/root/robot_config.yaml', pc_path: '/root/protocol_config.yaml'}"`

NEXT: Launch the action_client node automatically on container startup