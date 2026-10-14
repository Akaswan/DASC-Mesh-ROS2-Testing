# Multi-arch (amd64 + arm64), so the same image builds on the laptop and the Pi.
FROM ros:foxy-ros-base

WORKDIR /ws
COPY src/ src/
RUN . /opt/ros/foxy/setup.sh \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
