FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Harmonic, ROS 2 bridges, messages, RViz, and visualization stack (Xvfb + noVNC + Openbox)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-tf2-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-rviz2 \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-simple-commander \
    xvfb \
    openbox \
    x11vnc \
    novnc \
    websockify \
    mesa-utils \
    libgl1-mesa-dri \
    libqt5svg5 \
    qml-module-qtquick2 \
    qml-module-qtquick-controls2 \
    qml-module-qtquick-shapes \
    x11-utils \
    iputils-ping \
    net-tools \
    curl \
    procps \
    scrot \
    && rm -rf /var/lib/apt/lists/*

# Install API dependencies (FastAPI, Uvicorn, HTTPX, Pytest)
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-fastapi \
    python3-uvicorn \
    python3-httpx \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Link noVNC default index to vnc.html
RUN ln -sf /usr/share/novnc/vnc.html /usr/share/novnc/index.html

# Automatically source ROS 2 setup in interactive shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace/gazebo_four_wheel_robot

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
