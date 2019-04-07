FROM pittras/magellan-2018-base:master-41

# Add known ROS dependencies to the pittras/magellan-2018-docker-base repo
# This avoids rosdep redownloading them
# Remember to update the base image for this Dockerfile when you make changes!

COPY . /robot
WORKDIR /robot

RUN bash -c "source /opt/magellan-deps/devel/setup.bash && \
             rosdep install --from-paths src --ignore-src -y && \
             catkin_make && \
             echo \"source /opt/magellan-deps/devel/setup.bash\" >> ~/.bashrc && \
             echo \"source /robot/devel/setup.bash\" >> ~/.bashrc"
