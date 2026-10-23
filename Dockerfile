FROM robotis/open-manipulator:5.1.3@sha256:6d3c1b1510eb947eac01c3519b9d8ad2203205a69880fd6cdcd9b200fb88a8b8
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends python3-serial \
    && rm -rf /var/lib/apt/lists/*
COPY description/ /opt/replay_ws/src/open_manipulator_description/
RUN source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash \
    && cd /opt/replay_ws && colcon build --packages-select open_manipulator_description \
       --cmake-args -DCMAKE_BUILD_TYPE=Release
COPY *.py controllers.yaml entrypoint.sh /app/
RUN chmod +x /app/entrypoint.sh
ENV PYTHONDONTWRITEBYTECODE=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1
WORKDIR /app
ENTRYPOINT ["/app/entrypoint.sh"]
