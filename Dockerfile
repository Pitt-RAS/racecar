FROM pittras/magellan-2018-base:master-12

# Add known ROS dependencies to the pittras/magellan-2018-docker-base repo
# This avoids rosdep redownloading them
# Remember to update the base image for this Dockerfile when you make changes!

COPY . /robot
WORKDIR /robot

RUN bash -c "source /opt/ros/melodic/setup.bash && rosdep install --from-paths src --ignore-src -y"

RUN bash -c "source /opt/magellan-deps/devel/setup.bash && catkin_make"
RUN echo "source /opt/magellan-deps/devel/setup.bash" >> ~/.bashrc
RUN echo "source /robot/devel/setup.bash" >> ~/.bashrc
