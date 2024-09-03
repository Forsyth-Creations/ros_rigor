FROM ubuntu:22.04

ARG username=forsyth

# Install Bazel

WORKDIR /BAZEL_SETUP

RUN apt update && apt install apt-transport-https curl gnupg -y && \
curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor >bazel-archive-keyring.gpg && \
mv bazel-archive-keyring.gpg /usr/share/keyrings && \
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/bazel-archive-keyring.gpg] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list && \
apt update && apt install bazel -y

# Install Bazel Using Bazelisk
RUN apt update && apt install curl git -y
RUN apt install python3.11 python3.11-dev -y && mv /usr/bin/python3.11 /usr/bin/python3

RUN useradd -m -s /bin/bash linuxbrew && \
    useradd -m -s /bin/bash $username && \
    echo 'linuxbrew ALL=(ALL) NOPASSWD:ALL' >>/etc/sudoers

USER linuxbrew
RUN /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"

ENV PATH="/home/linuxbrew/.linuxbrew/bin:${PATH}"

# Add Brew to path

RUN brew install bazelisk

# add that file to the path
WORKDIR /WORKSPACE
COPY . . 
