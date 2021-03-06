# Docker container to easily test the robot-rest-api functionality
# At some point I will have to use docker compose to run multiple ros nodes
# Build with: sudo docker build -t robot_rest_api <robot-rest-api-location>/containers/Dockerfile
# Run with: sudo docker-compose up --remove-orphans
FROM ros:melodic-ros-core-bionic

LABEL maintainer="George Stavrinos <gstavrinos@iit.demokritos.gr>"

RUN apt-get update && apt-get install -y \
    apache2 \
    xvfb \
    mysql-server \
    python-catkin-tools \
    ros-melodic-costmap-2d \
    ros-melodic-costmap-queue \
    ros-melodic-octomap-mapping \
    ros-melodic-octomap-msgs \
    ros-melodic-octomap-ros \
    ros-melodic-octomap-server \
    ros-melodic-rosbridge-server \
    ros-melodic-tf2-web-republisher \
    ros-melodic-rospy-message-converter \
    ros-melodic-gazebo-ros-control \
    ros-melodic-gazebo-plugins && \
    rm -rf /var/lib/apt-get/lists/* 

RUN mkdir -p /opt/catkin_ws/src

RUN apt-get install -y \
        build-essential \
        git \
        python-rosdep && \
    rosdep init && \
    rosdep update && \
    cd /opt/catkin_ws/src && \
    git clone https://github.com/roboskel/roboskel_msgs && \
    git clone https://github.com/gstavrinos/mir_robot && \
    git clone https://github.com/gstavrinos/mir_experiments && \
    git clone https://github.com/gstavrinos/ros_rvv && \
    git clone https://github.com/carlosmccosta/laserscan_to_pointcloud && \
    git clone https://github.com/roboskel/roboskel_ros_resources && \
    git clone https://github.com/roboskel/robot-rest-api && \
    git clone https://github.com/osrf/gazebo_models && \
    cp -r robot-rest-api/web/* /var/www/html/ && \
    # rm -rf robot-rest-api && \
    cp -r mir_robot/mir_description /var/www/html && \
    cp -r roboskel_ros_resources /var/www/html && \
    mkdir -p ~/.gazebo/models && \
    cp -r gazebo_models/ambulance ~/.gazebo/models/ && \
    cp -r gazebo_models/cardboard_box ~/.gazebo/models/ && \
    cp -r gazebo_models/grey_tote ~/.gazebo/models/ && \
    cp -r gazebo_models/suv ~/.gazebo/models/ && \
    cp -r gazebo_models/plastic_cup ~/.gazebo/models/ && \
    cp -r gazebo_models/bowl ~/.gazebo/models/ && \
    cp -r gazebo_models/cordless_drill ~/.gazebo/models/ && \
    cp -r gazebo_models/coke_can ~/.gazebo/models/ && \
    rm -rf gazebo_models && \
    cp -r roboskel_ros_resources/gazebo/models/* ~/.gazebo/models && \
    rm -rf roboskel_ros_resources/graspit && \
    cd .. && \
    rosdep install --from-paths src --ignore-src -r -y && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin_make --pkg roboskel_msgs" && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash; source /opt/catkin_ws/devel/setup.bash; catkin_make" && \
    apt-get purge -y \
        build-essential \
        git && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt-get/lists/*

RUN sed --in-place --expression \
    '$isource "/opt/catkin_ws/devel/setup.bash"' \
    /ros_entrypoint.sh

RUN sh /opt/catkin_ws/src/robot-rest-api/rest/rust/install_all.sh

RUN /bin/bash -c ". /opt/ros/melodic/setup.bash && . /opt/catkin_ws/devel/setup.bash && . ~/.cargo/env && cd /opt/catkin_ws/src/robot-rest-api/rest/rust/robot-rest-api/ && cargo build"

EXPOSE 80
# EXPOSE 3306
# EXPOSE 11311/udp
# EXPOSE 11311/tcp

# ENTRYPOINT roslaunch mir_experiments changes_based_on_tidy_small_room_depth_camera.launch
# CMD ["roslaunch", "mir_experiments", "changes_based_on_tidy_small_room_depth_camera.launch"]

