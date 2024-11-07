FROM ubuntu:noble-20241011

ENV DEBIAN_FRONTEND=noninteractive  

# ------- Set up the locale -------

RUN locale && \
    apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# ---------- Enable Required Repositories ----------

RUN apt install software-properties-common -y && add-apt-repository universe

RUN apt update && apt install curl -y

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --------- Install the Dev tools ------------

RUN apt update && apt install ros-dev-tools -y

# --------- Install ROS 2 Jazzy ------------

RUN apt install ros-jazzy-desktop -y

# RUN apt update && apt install -y \
#     python3-flake8-blind-except \
#     python3-flake8-class-newline \
#     python3-flake8-deprecated \
#     python3-mypy \
#     python3-pip \
#     python3-pytest \
#     python3-pytest-cov \
#     python3-pytest-mock \
#     python3-pytest-repeat \
#     python3-pytest-rerunfailures \
#     python3-pytest-runner \
#     python3-pytest-timeout \
#     ros-dev-tools

# Set the working directory
# WORKDIR /ros2_jazzy

# # The latest release
# # https://github.com/ros2/ros2/releases/download/release-jazzy-20240919/ros2-jazzy-20240919-linux-noble-amd64.tar.bz2
# RUN curl -L https://github.com/ros2/ros2/releases/download/release-jazzy-20240919/ros2-jazzy-20240919-linux-noble-amd64.tar.bz2 --output ros2-package-linux-x86_64.tar.bz2

# RUN tar xf ros2-package-linux-x86_64.tar.bz2

# # Install dependencies using rosdep

# RUN apt upgrade && \
#     apt update && \
#     apt install -y python3-rosdep && \
#     rosdep init && \
#     rosdep update && \
#     rosdep install --from-paths /ros2_jazzy/ros2-linux/share --ignore-src -y --skip-keys "cyclonedds fastcdr fastrtps iceoryx_binding_c rmw_connextdds rti-connext-dds-6.0.1 urdfdom_headers"

# Add the source script to the bashrc
# RUN echo "source /ros2_jazzy/ros2-linux/setup.bash" >> ~/.bashrc

# Delete the tar file
# RUN rm ros2-package-linux-x86_64.tar.bz2


# ---------------- Install Gazebo Harmonic ----------------

# Install Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt update && apt install -y gz-harmonic


# -------- Installing things to make URDF work -----------------
RUN apt install ros-jazzy-urdf-tutorial \
    nano \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joy* \
    ros-jazzy-joint-state-publisher -y

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
# RUN echo "export GZ_SIM_RESOURCE_PATH=$GAZEBO_MODEL_PATH:/Robots" >> ~/.bashrc

ENV GZ_VERSION "harmonic"
ENV GZ_SIM_RESOURCE_PATH "$GAZEBO_MODEL_PATH:/Robots"

WORKDIR /ros2_jazzy/workspace
