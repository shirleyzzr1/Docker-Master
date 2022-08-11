FROM kwelbeck/base-ros2-with-empty-overlay:latest

# Copying config files
WORKDIR /root
COPY resources/ .

# Downloading ros packages and Creating an overlay
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS/src
COPY ros-packages .
RUN vcs import < repos
WORKDIR $ROS_WS
SHELL ["/bin/bash", "-c"]
RUN source $ROS_ROOT/setup.bash && colcon build --symlink-install && source $ROS_WS/install/setup.bash

# Copying in entrypoint and intiator scripts
COPY ["execute.sh", "ros_entrypoint.sh", "/"]

# On image run, source overlay and launch node
ENTRYPOINT ["/ros_entrypoint.sh"]
#CMD ["ros2", "run", "demo", "action_client"]
CMD ros2 launch demo demo.launch.py
