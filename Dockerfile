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

# --------- Install the Dev tools, ROS Base and RViz2 ---------

RUN apt update && apt install ros-dev-tools \
    ros-jazzy-ros-base \
    ros-jazzy-rviz2 \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rqt-tf-tree \
    nano \
    ros-jazzy-urdf-tutorial \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-nav2-minimal-tb* \
    -y

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# ---------------- Install Gazebo Ionic ----------------

# Install Gazebo Ionic
ENV GZ_VERSION "harmonic"
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt update && \
    apt install -y gz-harmonic



#  ----------------------- Install Node.js -----------------------
ENV NVM_DIR=/root/.nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.38.0/install.sh | bash && \
    . $NVM_DIR/nvm.sh && \
    nvm install 22 && \
    nvm alias default 22 && \
    nvm use 22

# Add nvm to PATH for subsequent RUN instructions
ENV PATH=$NVM_DIR/versions/node/v22/bin:$PATH

# ----------------------- Install Poetry and Python Venv -----------------------

# Install Poetry for dependency management
RUN curl -sSL https://install.python-poetry.org | python3 -

# Add poetry to PATH
ENV PATH="/root/.local/bin:$PATH"

RUN poetry --version

RUN apt update && apt install python3-pip python3.12-venv  -y

# Create a venv for the workspace
RUN python3.12 -m venv /venv

# Add it to the bashrc
RUN echo "source /venv/bin/activate" >> ~/.bashrc

# pip install catkin-pkg after activating the venv
RUN /venv/bin/pip install catkin-pkg pyyaml

# make a home command to go to the workspace
RUN echo "home () { cd /ros2_jazzy/workspace; }" >> ~/.bashrc

ENV GZ_SIM_RESOURCE_PATH "/ros2_jazzy/workspace"

WORKDIR /ros2_jazzy/workspace
