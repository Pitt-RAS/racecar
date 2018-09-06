FROM ros:melodic-robot

# Add known dependencies here to cache them
RUN apt-get update
RUN apt-get install -y ros-melodic-robot-localization

COPY . /robot
WORKDIR /robot

RUN bash -c "source /opt/ros/melodic/setup.bash && rosdep install --from-paths src --ignore-src -y"

RUN bash -c "source /opt/ros/melodic/setup.bash && catkin_make"
RUN echo "source /robot/devel/setup.bash" >> ~/.bashrc
