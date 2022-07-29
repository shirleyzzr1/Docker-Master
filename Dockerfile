FROM kwelbeck/base-ros2-with-empty-overlay:latest

# copying config files
WORKDIR /root
COPY resources/robot_config.yaml /root/robot_config.yaml
COPY resources/protocol_config.yaml /root/protocol_config.yaml

# creating, downloading resource directories ros packages and sourcing an overlay
RUN mkdir -p $ROS_WS/src/demo
    
WORKDIR $ROS_WS
COPY demo src/demo/
RUN git clone -b demo https://github.com/kjwelbeck3/demo_interfaces.git \
    && mv demo_interfaces src

SHELL ["/bin/bash", "-c"]
RUN source $ROS_ROOT/setup.bash && colcon build --symlink-install && source $ROS_WS/install/setup.bash

#setup default running script
COPY ./execute.sh /

# setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "run", "demo", "action_client"]
