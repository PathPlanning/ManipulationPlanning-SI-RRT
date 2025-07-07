# Installation and Dependencies

This document provides detailed instructions for installing and setting up the ManipulationPlanning-SI-RRT framework on different platforms.

## Docker Setup (Alternative)

If you prefer to use Docker exclusively, you can skip most of the above steps and simply pull our pre-built Docker image:

```bash
make pull_docker
```

This will download a containers with all the necessary dependencies pre-installed:

- mdp_release: all code in this docker built for maximum performance
- mdp_debug: all code in this docker built with Debug support


## Dependencies

## Installation Steps

### Linux (Ubuntu 24.04)

1. **Install basic dependencies**:
   ```bash
   sudo apt update
   sudo apt install -y cmake build-essential python3-dev python3-pip libboost-all-dev swig htop nano curl lsb-release git
   ```

2. **Install ROS2**:

Refer to https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
In theory, any ros2 distribution is enough, but tested only for jazzy and humble.

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo apt install /tmp/ros2-apt-source.deb
```

```bash
sudo apt update && sudo apt install ros-dev-tools
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
```


```bash
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash">> ~/.bashrc
```

3. **Install DRGBT dependencies**:

```bash
sudo apt-get update 
sudo apt-get install  -y libgtest-dev libnanoflann-dev libpcl-dev libeigen3-dev libgflags-dev libgoogle-glog-dev  libyaml-cpp-dev  libfcl-dev

# if necessary
sudo apt-get install  -y liburdf-dev liborocos-kdl-dev libkdl-parser-dev 
```

4. **Install coal and ompl dependencies + python libraries** :

```bash
sudo apt-get update
sudo apt-get install  -y qhull-bin octomap-tools castxml  python3-rosdep
```

```bash
sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg' >> /etc/apt/sources.list.d/robotpkg.list" 
sudo curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add - 
sudo apt-get update 
sudo apt install -y robotpkg-py*-eigenpy 

export eigenpy_DIR=/opt/openrobots

echo "export eigenpy_DIR=/opt/openrobots" >> ~/.bashrc
```

```bash 
pip3 install --upgrade pip
pip3 install -U wheel eigenpy numpy castxml pybullet matplotlib scipy pygccxml pyplusplus
```

5. Install RapidJSON

```bash
git clone --recursive https://github.com/Tencent/rapidjson.git
cd rapidjson
mkdir build
cd build
cmake .. -DRAPIDJSON_BUILD_DOC=OFF -DRAPIDJSON_BUILD_EXAMPLES=OFF  -DRAPIDJSON_BUILD_TESTS=OFF
make install
cd ../.. 
```

6. **Install Coal:**

```bash
git clone --recursive https://github.com/coal-library/coal 
cd coal 
rosdep init
rosdep update
rosdep install --from-paths ./ -y --ignore-src
mkdir build  
cd build 
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
-DCOAL_HAS_QHULL=True \
-DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=True \ 
-DCMAKE_C_FLAGS_RELEASE="-march=native -mtune=native -O3 -fPIC -flto=auto -fomit-frame-pointer -fdata-sections -ffunction-sections -Wl,--gc-sections" \
-DCMAKE_CXX_FLAGS_RELEASE="-march=native -mtune=native -O3 -fPIC -flto=auto -fomit-frame-pointer -fdata-sections -ffunction-sections -Wl,--gc-sections" 
make -j120
sudo make install -j120
cd ../.. 
```

7. **Install OMPL:**

```bash
git clone --recursive https://github.com/ompl/ompl.git 
cd ompl 
mkdir -p build 
cd build 
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release \
-DCMAKE_C_FLAGS_RELEASE="-march=native -mtune=native -O3 -fPIC -flto=auto -fomit-frame-pointer -fdata-sections -ffunction-sections -Wl,--gc-sections" \
-DCMAKE_CXX_FLAGS_RELEASE="-march=native -mtune=native -O3 -fPIC -flto=auto -fomit-frame-pointer -fdata-sections -ffunction-sections -Wl,--gc-sections "  
make -j128
make install 
cd ../.. 
```

8. **Install Blender:**

Download blender-*.tar.gz from https://www.blender.org/download/ , extract it somewhere, and add symbolic link

```bash
sudo ls -s \usr\bin\blender \path_to_extracted_belnder\blender
```

9. **Fix header and linker error:**

```bash
export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/opt/ros/jazzy/include/urdf:/opt/ros/jazzy/include/urdfdom_headers/:/opt/ros/jazzy/include/kdl_parser
sudo ln -s /opt/ros/jazzy/lib/liburdf.so /usr/local/lib/liburdf.so
sudo ln -s /opt/ros/jazzy/lib/libkdl_parser.so /usr/local/lib/libkdl_parser.so

echo "export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/opt/ros/jazzy/include/urdf:/opt/ros/jazzy/include/urdfdom_headers/:/opt/ros/jazzy/include/kdl_parser" >> ~/.bashrc
```