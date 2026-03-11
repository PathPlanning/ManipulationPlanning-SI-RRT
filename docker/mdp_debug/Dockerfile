FROM ubuntu:24.04

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
    gdb \ 
    valgrind \
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
    apt install -y ros-jazzy-desktop && \
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc 



#install DRGBT dependencies
RUN apt-get update && apt-get install  -y \
    libgtest-dev \
    libnanoflann-dev \
    libpcl-dev \
    libeigen3-dev \
    # libkdl-parser-dev \
    ros-jazzy-kdl-parser \
    libgflags-dev \
    libgoogle-glog-dev \
    liborocos-kdl-dev \
    libyaml-cpp-dev \
    # liburdf-dev \
    ros-jazzy-urdf \
    libfcl-dev && \
    rm -rf /var/lib/apt/lists/* 

# #install eigenpy for ompl and coal
RUN sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list" && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key |  apt-key add - && \
    apt-get update && \
    apt install -y robotpkg-py*-eigenpy 

ENV eigenpy_DIR=/opt/openrobots

RUN apt-get update && apt-get install -y \
    qhull-bin \
    octomap-tools \
    castxml \
    python3-rosdep \
    python3-venv \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip wheel && \
    /opt/venv/bin/pip install \
        eigenpy \
        numpy \
        castxml \
        pybullet \
        matplotlib \
        scipy \
        pygccxml \
        pyplusplus

ENV VIRTUAL_ENV=/opt/venv
ENV PATH="/opt/venv/bin:$PATH"
RUN echo 'source /opt/venv/bin/activate' >> /root/.bashrc

# commits hash for reproducability
ARG COAL_COMMIT=ff01445364a7b386e8e5c46ccd530ed9d9a8e8cf
ARG RAPIDJSON_COMMIT=24b5e7a8b27f42fa16b96fc70aade9106cf7102f
ARG OMPL_COMMIT=f2e2ec3de7698063fa18d9c5f676bc9dcefc0344
ARG OMPL_COMMIT_before_fix=05aa440f896ee08b34e8ffe83c7b5a9db5434789

# Install hpp-fcl (coal)
RUN apt-get update && \
    mkdir ./libs && cd ./libs && \
    git clone --recursive https://github.com/coal-library/coal  && \
    cd coal  && \
    git checkout --detach ${COAL_COMMIT} && \
    git submodule update --init --recursive && \
    . /opt/ros/jazzy/setup.sh && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths ./ -y --ignore-src && \
    mkdir build  && \ 
    cd build  && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Debug \
    -DHPP_FCL_HAS_QHULL=True \
    -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=True \ 
    -DCMAKE_C_FLAGS_DEBUG="-g -O0" \
    -DCMAKE_CXX_FLAGS_DEBUG="-g -O0"  && \
    make -j4 && \
    cmake --install .  && \
    cd ../..  


    # Install RapidJSON
RUN git clone --recursive https://github.com/Tencent/rapidjson.git && \
    cd rapidjson && \
    git checkout --detach ${RAPIDJSON_COMMIT} && \
    git submodule update --init --recursive && \
    mkdir build && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_FLAGS_DEBUG="-g -O0" \
    -DCMAKE_CXX_FLAGS_DEBUG="-g -O0" && \ 
    make -j4 && \ 
    make install && \
    cd ../.. && \
    cd / 

    #install OMPL 
RUN git clone --recursive https://github.com/PathPlanning/ompl/tree/Strrt_before_memory_leak && \
    cd ompl && \
    mkdir -p build && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_FLAGS_DEBUG="-g -O0" \
    -DCMAKE_CXX_FLAGS_DEBUG="-g -O0"  && \
    make -j4 && \ 
    make install update_bindings && \
    cd ../.. && \
    #install new OMPL, where ST-RRT* memory leak is fixed (for research purposes) 
    git clone --recursive https://github.com/ompl/ompl.git ompl-new && \
    cd ompl-new && \
    git checkout --detach ${OMPL_COMMIT} && \
    git submodule update --init --recursive && \
    mkdir -p build && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ompl-new -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_FLAGS_DEBUG="-g -O0" \
    -DCMAKE_CXX_FLAGS_DEBUG="-g -O0"  && \
    make -j4 && \ 
    make install update_bindings && \
    cd ../.. && \
    git clone --recursive https://github.com/ompl/ompl.git ompl-old&& \
    cd ompl-old && \
    git checkout --detach ${OMPL_COMMIT_before_fix} && \
    git submodule update --init --recursive && \
    mkdir -p build && \
    cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/opt/ompl-old -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_C_FLAGS_DEBUG="-g -O0" \
    -DCMAKE_CXX_FLAGS_DEBUG="-g -O0"  && \
    make -j4 && \ 
    make install update_bindings && \
    cd ../.. 

ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

RUN ln -s /usr/local/lib/libcoal.so /usr/local/lib/libhpp-fcl.so

RUN apt install -y blender

SHELL ["/bin/bash", "-o"]



