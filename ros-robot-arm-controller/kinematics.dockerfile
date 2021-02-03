FROM ros:melodic-robot
RUN mkdir -p /home/catkin_ws/src 
WORKDIR /home/catkin_ws
RUN . /opt/ros/melodic/setup.sh && catkin_make
RUN bash -c "echo \"source /home/catkin_ws/devel/setup.bash\" >> ~/.bashrc"