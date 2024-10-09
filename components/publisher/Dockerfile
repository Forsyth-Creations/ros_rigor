# ROS2 Python
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# Installing ROS2 Jazzy

RUN apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    locale

RUN apt install software-properties-common -y && \
    add-apt-repository universe && \
    apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install Dev Tools
RUN apt update && apt install ros-dev-tools -y


# Install ROS2
RUN apt update && apt upgrade -y

RUN apt install ros-jazzy-desktop -y

# Copy in the code

WORKDIR /publisher


# Install PIP
RUN apt install python3-pip python3.12-venv -y

# Create a virtual environment
RUN python3 -m venv venv

# Install poetry
RUN . venv/bin/activate && pip install poetry==1.8.3

# Update .bashrc to include the virtual environment
RUN echo "source /publisher/venv/bin/activate" >> ~/.bashrc

# Also add the ros2 setup to the bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

COPY pyproject.toml poetry.lock ./

# Install dependencies
RUN . venv/bin/activate && poetry config virtualenvs.create false --local && poetry install

# Copy everything else
COPY . .
    

