# syntax=docker/dockerfile:1.7

ARG ROS_DISTRO=jazzy
ARG UBUNTU_CODENAME=noble

FROM ros:${ROS_DISTRO}-ros-base-${UBUNTU_CODENAME} AS builder

SHELL ["/bin/bash", "-euxo", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO

# Third-party refs
ARG COAL_COMMIT=ff01445364a7b386e8e5c46ccd530ed9d9a8e8cf
ARG OMPL_LEGACY_REF=Strrt_before_memory_leak
ARG RAPIDJSON_COMMIT=24b5e7a8b27f42fa16b96fc70aade9106cf7102f
ARG NANOFLANN_REF=v1.9.0

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    CMAKE_PREFIX_PATH=/opt/vendor:/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH} \
    CPLUS_INCLUDE_PATH=/opt/vendor/include:/opt/ros/${ROS_DISTRO}/include:/opt/ros/${ROS_DISTRO}/include/urdf:/opt/ros/${ROS_DISTRO}/include/urdfdom_headers:/opt/ros/${ROS_DISTRO}/include/kdl_parser:${CPLUS_INCLUDE_PATH} \
    LIBRARY_PATH=/opt/vendor/lib:/opt/ros/${ROS_DISTRO}/lib:${LIBRARY_PATH} \
    LD_LIBRARY_PATH=/opt/vendor/lib:/opt/ros/${ROS_DISTRO}/lib:${LD_LIBRARY_PATH}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
      locales \
      ca-certificates \
      curl \
      git \
      make \
      build-essential \
      cmake \
      ninja-build \
      pkg-config \
      file \
      python3 \
      python3-dev \
      python3-venv \
      python3-pip \
      swig \
      castxml \
      qhull-bin \
      libqhull-dev \
      octomap-tools \
      liboctomap-dev \
      libboost-all-dev \
      libeigen3-dev \
      libgflags-dev \
      libgoogle-glog-dev \
      liborocos-kdl-dev \
      libyaml-cpp-dev \
      libfcl-dev \
      libccd-dev \
      liburdfdom-dev \
      libassimp-dev \
      ros-${ROS_DISTRO}-kdl-parser \
      ros-${ROS_DISTRO}-urdf \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src

# RPMPLv2 must already exist in the build context
COPY STRRT_Planner ./STRRT_Planner
COPY MSIRRT ./MSIRRT
COPY RPMPLv2 ./RPMPLv2

# Remove stale local build artifacts copied from host, then patch sources only
RUN find /src/STRRT_Planner /src/MSIRRT /src/RPMPLv2 \
      -type d \( \
        -name build -o \
        -name build_release -o \
        -name build_debug -o \
        -name 'cmake-build-*' \
      \) -prune -exec rm -rf {} + && \
    find /src/STRRT_Planner /src/MSIRRT /src/RPMPLv2 \
      -type f \( \
        -name compile_commands.json -o \
        -name CMakeCache.txt \
      \) -delete && \
    find /src/STRRT_Planner /src/MSIRRT /src/RPMPLv2 \
      \( -name Makefile -o -name CMakeLists.txt \) -type f -print0 \
    | xargs -0 sed -Ei \
      -e 's/-march=native[[:space:]]*//g' \
      -e 's/-mtune=native[[:space:]]*//g' && \
    ! find /src/STRRT_Planner /src/MSIRRT /src/RPMPLv2 \
        \( -name Makefile -o -name CMakeLists.txt \) -type f -print0 \
      | xargs -0 grep -nE -- '-march=native|-mtune=native'

# Build coal / hpp-fcl-compatible runtime into /opt/vendor
RUN git clone --filter=blob:none --recursive https://github.com/coal-library/coal.git /tmp/coal && \
    git -C /tmp/coal checkout --detach "${COAL_COMMIT}" && \
    git -C /tmp/coal submodule update --init --recursive && \
    cmake -S /tmp/coal -B /tmp/coal/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/vendor \
      -DHPP_FCL_HAS_QHULL=ON \
      -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON \
      -DBUILD_TESTING=OFF \
      -DBUILD_PYTHON_INTERFACE=OFF \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DPython_EXECUTABLE=/usr/bin/python3 \
      -DPython3_EXECUTABLE=/usr/bin/python3 && \
    cmake --build /tmp/coal/build --parallel "$(nproc)" && \
    cmake --install /tmp/coal/build && \
    ln -sf /opt/vendor/lib/libcoal.so /opt/vendor/lib/libhpp-fcl.so && \
    rm -rf /tmp/coal

# Build OMPL branch used by STRRT into /opt/vendor
RUN git clone --filter=blob:none --recursive --branch "${OMPL_LEGACY_REF}" \
      https://github.com/PathPlanning/ompl.git /tmp/ompl && \
    git -C /tmp/ompl submodule update --init --recursive && \
    cmake -S /tmp/ompl -B /tmp/ompl/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/vendor \
      -DOMPL_BUILD_DEMOS=OFF \
      -DOMPL_BUILD_PYBINDINGS=OFF \
      -DBUILD_TESTING=OFF && \
    cmake --build /tmp/ompl/build --parallel "$(nproc)" && \
    cmake --install /tmp/ompl/build && \
    rm -rf /tmp/ompl

# Build RapidJSON from source into /opt/vendor
RUN git clone --filter=blob:none --recursive https://github.com/Tencent/rapidjson.git /tmp/rapidjson && \
    git -C /tmp/rapidjson checkout --detach "${RAPIDJSON_COMMIT}" && \
    git -C /tmp/rapidjson submodule update --init --recursive && \
    cmake -S /tmp/rapidjson -B /tmp/rapidjson/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/vendor \
      -DRAPIDJSON_BUILD_DOC=OFF \
      -DRAPIDJSON_BUILD_EXAMPLES=OFF \
      -DRAPIDJSON_BUILD_TESTS=OFF && \
    cmake --build /tmp/rapidjson/build --parallel "$(nproc)" && \
    cmake --install /tmp/rapidjson/build && \
    rm -rf /tmp/rapidjson

# Build nanoflann from source into /opt/vendor
RUN git clone --filter=blob:none --recursive https://github.com/jlblancoc/nanoflann.git /tmp/nanoflann && \
    git -C /tmp/nanoflann checkout --detach "${NANOFLANN_REF}" && \
    git -C /tmp/nanoflann submodule update --init --recursive && \
    cmake -S /tmp/nanoflann -B /tmp/nanoflann/build -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/opt/vendor \
      -DNANOFLANN_BUILD_EXAMPLES=OFF \
      -DNANOFLANN_BUILD_TESTS=OFF && \
    cmake --build /tmp/nanoflann/build --parallel "$(nproc)" && \
    cmake --install /tmp/nanoflann/build && \
    rm -rf /tmp/nanoflann

# Build planners from copied source
RUN cd /src/STRRT_Planner && make build_release
RUN cd /src/MSIRRT && make build_release
RUN cd /src/RPMPLv2 && make build

# Strip our built artifacts
RUN find /opt/vendor/lib -maxdepth 1 -type f -name '*.so*' -exec strip --strip-unneeded {} + || true && \
    strip --strip-unneeded \
      /src/STRRT_Planner/build/STRRT_Planner \
      /src/MSIRRT/build/MSIRRT_Planner \
      /src/RPMPLv2/build/src/main || true

# Collect planner binaries + exact runtime shared libraries
RUN cat >/usr/local/bin/collect-runtime.sh <<'EOF' && chmod +x /usr/local/bin/collect-runtime.sh
#!/usr/bin/env bash
set -euo pipefail

stage=/opt/stage
mkdir -p "$stage/opt/planners/bin" "$stage/opt/planners/meta"

copy_abs() {
  local p="$1"
  [ -e "$p" ] || return 0
  cp --parents -L "$p" "$stage"
}

deps_of() {
  ldd "$1" | awk '
    /=>/ && $(NF-1) ~ /^\// { print $(NF-1) }
    /^\// { print $1 }
  ' | sort -u
}

for bin in "$@"; do
  name="$(basename "$bin")"
  install -Dm755 "$bin" "$stage/opt/planners/bin/$name"

  {
    echo "### $bin"
    ldd "$bin" || true
    echo
  } >> "$stage/opt/planners/meta/ldd-report.txt"

  while read -r lib; do
    [ -n "$lib" ] || continue
    copy_abs "$lib"
  done < <(deps_of "$bin")
done
EOF

RUN /usr/local/bin/collect-runtime.sh \
      /src/STRRT_Planner/build/STRRT_Planner \
      /src/MSIRRT/build/MSIRRT_Planner \
      /src/RPMPLv2/build/src/main && \
    mv /opt/stage/opt/planners/bin/main /opt/stage/opt/planners/bin/RPMPLv2_main && \
    printf '%s\n' \
      STRRT_Planner \
      MSIRRT_Planner \
      RPMPLv2_main \
      > /opt/stage/opt/planners/meta/planners.txt

# If RPMPLv2 needs runtime assets, uncomment:
# RUN mkdir -p /opt/stage/opt/planners/share/RPMPLv2 && \
#     cp -a /src/RPMPLv2/build/apps/data /opt/stage/opt/planners/share/RPMPLv2/ && \
#     cp -a /src/RPMPLv2/build/apps/visualizer /opt/stage/opt/planners/share/RPMPLv2/

FROM ubuntu:24.04 AS runtime

SHELL ["/bin/bash", "-euxo", "pipefail", "-c"]

ARG ROS_DISTRO=jazzy

ENV PATH=/opt/planners/bin:${PATH} \
    LD_LIBRARY_PATH=/opt/vendor/lib:/opt/ros/${ROS_DISTRO}/lib:${LD_LIBRARY_PATH}

COPY --from=builder /opt/stage/ /

RUN printf '%s\n' \
      /opt/vendor/lib \
      /opt/ros/${ROS_DISTRO}/lib \
      > /etc/ld.so.conf.d/mdp-planners.conf && \
    ldconfig && \
    groupadd --gid 1000 app && \
    useradd --uid 1000 --gid 1000 --create-home --shell /bin/bash app && \
    mkdir -p /workspace && chown app:app /workspace

WORKDIR /workspace
USER app
CMD ["bash"]