FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    python3-dev \
    python3-pip \
    libboost-all-dev \
    swig \
    htop \
    nano \
    curl \
    lsb-release \
    git && \
    rm -rf /var/lib/apt/lists/* 
#install ros2
RUN locale && \
    apt update && apt install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 

ENV LANG=en_US.UTF-8

RUN apt install -y software-properties-common && \
    add-apt-repository universe && \
    apt update && apt install -y curl  && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt update && \
    apt upgrade -y && \
    apt install -y ros-humble-desktop && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 



#install DRGBT dependencies
RUN apt-get update && apt-get install  -y \
    libgtest-dev \
    libnanoflann-dev \
    libpcl-dev \
    libeigen3-dev \
    libkdl-parser-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liborocos-kdl-dev \
    libyaml-cpp-dev \
    liburdf-dev \
    libfcl-dev && \
    rm -rf /var/lib/apt/lists/* 

# #install eigenpy for ompl

# RUN sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list" && \
#     curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key |  apt-key add - && \
#     apt-get update && \
#     apt install -y robotpkg-py*-eigenpy 

# ENV eigenpy_DIR=/opt/openrobots


RUN apt-get update && apt-get install  -y \ 
    # install qhull and octomap for fcl and castxml for ompl
    qhull-bin octomap-tools castxml  python3-rosdep    && \
    #  Python bindings for OMPL
    pip3 install --upgrade pip && \
    pip3 install -U wheel eigenpy numpy castxml pybullet matplotlib scipy pygccxml pyplusplus 




# Install RapidJSON
# cant do full optimisation here due to https://github.com/Tencent/rapidjson/issues/1924
RUN git clone --recursive https://github.com/Tencent/rapidjson.git && \
    cd rapidjson && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS_RELEASE="-O2" \
    -DCMAKE_CXX_FLAGS_RELEASE="-O2" && \ 
    make -j4 && \
    make install && \
    cd ../..  && \

    # Install coal
    git clone --recursive https://github.com/humanoid-path-planner/coal.git  && \
    cd coal  && \
    . /opt/ros/humble/setup.sh && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths ./ -y --ignore-src && \
    mkdir build  && \ 
    cd build  && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
    -DHPP_FCL_HAS_QHULL=True \
    -DCMAKE_C_FLAGS_RELEASE="-O3" \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3"  && \
    make -j4  && \
    make install  && \
    cd ../..  && \

    #install OMPL
    git clone --recursive https://github.com/ompl/ompl.git && \
    cd ompl && \
    mkdir -p build && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_FLAGS_RELEASE="-O3" \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3"  && \
    make -j4 && \ 
    make install update_bindings && \
    cd ../.. 



ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

RUN apt install -y blender



SHELL ["/bin/bash", "-o"]


