#!/usr/bin/env bash
set -Eeuo pipefail

# Ubuntu 24.04 + ROS 2 Jazzy installer/builder for
# STRRT_Planner, MSIRRT and RPMPLv2.


ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DEPS_ROOT="${DEPS_ROOT:-/opt/SIRRT_dependencies}"
SRC_ROOT="${DEPS_ROOT}/src"
RELEASE_PREFIX="${DEPS_ROOT}/release"
DEBUG_PREFIX="${DEPS_ROOT}/debug"
FORCE_REBUILD="${FORCE_REBUILD:-0}"

COAL_COMMIT="${COAL_COMMIT:-ff01445364a7b386e8e5c46ccd530ed9d9a8e8cf}"
RAPIDJSON_COMMIT="${RAPIDJSON_COMMIT:-24b5e7a8b27f42fa16b96fc70aade9106cf7102f}"
OMPL_COMMIT_2_0="${OMPL_COMMIT_2_0:-96b7b9c2c62f80570b5b93e03a95d56dea8410d9}"
NANOFLANN_COMMIT="${NANOFLANN_COMMIT:-ba47cfcb127c3597d69196d87f5aa9ca8811b0a9}"

RELEASE_C_FLAGS="${RELEASE_C_FLAGS:--march=native -mtune=native -O3 -DNDEBUG -fno-plt}"
RELEASE_CXX_FLAGS="${RELEASE_CXX_FLAGS:--march=native -mtune=native -O3 -DNDEBUG -fno-plt}"
DEBUG_C_FLAGS="${DEBUG_C_FLAGS:--O0 -g -fno-omit-frame-pointer}"
DEBUG_CXX_FLAGS="${DEBUG_CXX_FLAGS:--O0 -g -fno-omit-frame-pointer}"

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

log() {
  printf '\n\033[1;32m==> %s\033[0m\n' "$*"
}

fail() {
  printf '\n\033[1;31m[error]\033[0m %s\n' "$*" >&2
  exit 1
}

need_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "Missing command: $1"
}

require_ubuntu_24_04() {
  [[ -r /etc/os-release ]] || fail "/etc/os-release not found"
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" ]] || fail "Ubuntu required, got: ${ID:-unknown}"
  [[ "${VERSION_ID:-}" == "24.04" ]] || fail "Ubuntu 24.04 required, got: ${VERSION_ID:-unknown}"
}

source_ros_setup() {
  local old_opts="$-"

  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
  export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}"
  export COLCON_TRACE="${COLCON_TRACE:-}"
  export COLCON_PREFIX_PATH="${COLCON_PREFIX_PATH:-}"
  export COLCON_PYTHON_EXECUTABLE="${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}"
  export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"

  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  case "$old_opts" in
    *u*) set -u ;;
  esac
}

ensure_locale() {
  log "Configuring locale"
  ${SUDO} apt-get update -y
  ${SUDO} apt-get install -y locales
  if ! locale -a | grep -qi '^en_US\.utf8$'; then
    ${SUDO} locale-gen en_US en_US.UTF-8
  fi
  ${SUDO} update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 || true
  export LANG=en_US.UTF-8
  export LC_ALL=en_US.UTF-8
}

ensure_ros_repo() {
  if apt-cache show "ros-${ROS_DISTRO}-urdf" >/dev/null 2>&1; then
    log "ROS apt repo already available"
    return
  fi

  log "Adding ROS 2 apt repo"
  ${SUDO} apt-get update -y
  ${SUDO} apt-get install -y software-properties-common curl ca-certificates
  ${SUDO} add-apt-repository -y universe

  local ubuntu_codename
  local pool_url
  local pool_index
  local deb_name
  local deb_path

  # shellcheck disable=SC1091
  . /etc/os-release
  ubuntu_codename="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"

  pool_url="https://repo.ros2.org/ubuntu/main/pool/main/r/ros-apt-source/"
  deb_path="/tmp/ros2-apt-source.deb"

  log "Finding latest ros2-apt-source for ${ubuntu_codename}"

  pool_index="$(
    curl -fsSL --retry 5 --retry-delay 2 --retry-all-errors "$pool_url"
  )" || fail "Failed to read ROS package pool: ${pool_url}"

  deb_name="$(
    printf '%s\n' "$pool_index" \
      | grep -oE "ros2-apt-source_[0-9][^\"'<> ]*~${ubuntu_codename}_all\.deb" \
      | sort -V \
      | tail -n1
  )"

  [[ -n "$deb_name" ]] || fail "Could not find ros2-apt-source package for Ubuntu ${ubuntu_codename}"

  log "Downloading ${deb_name}"

  curl -fL --retry 5 --retry-delay 2 --retry-all-errors \
    -o "$deb_path" \
    "${pool_url}${deb_name}" \
    || fail "Failed to download ${pool_url}${deb_name}"

  log "Installing ros2-apt-source"
  ${SUDO} apt-get install -y "$deb_path"
  rm -f "$deb_path"

  log "Updating apt after ROS repo install"
  ${SUDO} apt-get update -y

  apt-cache show "ros-${ROS_DISTRO}-urdf" >/dev/null 2>&1 \
    || fail "ROS repo installed, but ros-${ROS_DISTRO}-urdf is still not visible to apt"
}

install_apt_packages() {
  local pkgs=(
    ca-certificates curl git make build-essential cmake ninja-build pkg-config file
    locales software-properties-common lsb-release
    python3 python3-dev python3-pip python3-venv python3-rosdep
    swig castxml
    qhull-bin libqhull-dev
    octomap-tools liboctomap-dev
    libboost-all-dev libgtest-dev libpcl-dev libeigen3-dev
    libgflags-dev libgoogle-glog-dev liborocos-kdl-dev libyaml-cpp-dev
    libfcl-dev libccd-dev liburdfdom-dev libassimp-dev
    "ros-${ROS_DISTRO}-ament-cmake"
    "ros-${ROS_DISTRO}-ament-cmake-core"
    "ros-${ROS_DISTRO}-ament-cmake-libraries"
    "ros-${ROS_DISTRO}-kdl-parser"
    "ros-${ROS_DISTRO}-urdf"
  )

  local missing=()
  local pkg
  for pkg in "${pkgs[@]}"; do
    if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q 'ok installed'; then
      missing+=("$pkg")
    fi
  done

  if ((${#missing[@]})); then
    log "Installing apt packages: ${missing[*]}"
    ${SUDO} apt-get update -y
    ${SUDO} apt-get install -y --no-install-recommends "${missing[@]}"
  else
    log "All apt packages are already installed"
  fi
}

ensure_rosdep() {
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initializing rosdep"
    ${SUDO} rosdep init || true
  fi
  rosdep update || true
}

ensure_dirs() {
  log "Preparing ${DEPS_ROOT}"
  ${SUDO} mkdir -p "$SRC_ROOT" "$RELEASE_PREFIX" "$DEBUG_PREFIX"
  ${SUDO} chown -R "${SUDO_USER:-$USER}:${SUDO_USER:-$USER}" "$DEPS_ROOT" || true
}

clone_or_update() {
  local name="$1"
  local url="$2"
  local ref="$3"
  local dst="${SRC_ROOT}/${name}"

  if [[ "$FORCE_REBUILD" == "1" ]]; then
    rm -rf "$dst"
  fi

  if [[ ! -d "$dst/.git" ]]; then
    log "Cloning ${name}"
    git clone --filter=blob:none --recursive "$url" "$dst"
  else
    log "Updating ${name}"
    git -C "$dst" fetch --all --tags --force
  fi

  if git -C "$dst" ls-remote --exit-code --heads origin "$ref" >/dev/null 2>&1; then
    git -C "$dst" checkout -B "$ref" "origin/$ref"
  else
    git -C "$dst" checkout --detach "$ref"
  fi
  git -C "$dst" submodule update --init --recursive
}

write_rapidjson_fallback_config() {
  local prefix="$1"
  local cfg_dir="${prefix}/lib/cmake/RapidJSON"
  local cfg_file="${cfg_dir}/RapidJSONConfig.cmake"
  mkdir -p "$cfg_dir"
  cat > "$cfg_file" <<EOF_RJ
set(RapidJSON_FOUND TRUE)
set(RapidJSON_INCLUDE_DIRS "${prefix}/include")
if(NOT TARGET RapidJSON)
  add_library(RapidJSON INTERFACE IMPORTED)
  set_target_properties(RapidJSON PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${prefix}/include"
  )
endif()
EOF_RJ
}

write_nanoflann_fallback_config() {
  local prefix="$1"
  local cfg_dir="${prefix}/lib/cmake/nanoflann"
  local cfg_file="${cfg_dir}/nanoflannConfig.cmake"
  mkdir -p "$cfg_dir"
  cat > "$cfg_file" <<EOF_NF
set(nanoflann_FOUND TRUE)
if(NOT TARGET nanoflann::nanoflann)
  add_library(nanoflann::nanoflann INTERFACE IMPORTED)
  set_target_properties(nanoflann::nanoflann PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${prefix}/include"
  )
endif()
EOF_NF
}

build_header_only_dep() {
  local name="$1"
  local ref="$2"
  local url="$3"
  shift 3
  local extra_args=("$@")

  clone_or_update "$name" "$url" "$ref"

  local src_dir="${SRC_ROOT}/${name}"
  local build_release="${src_dir}/build_release"
  local build_debug="${src_dir}/build_debug"

  if [[ "$FORCE_REBUILD" == "1" ]]; then
    rm -rf "$build_release" "$build_debug"
  fi

  log "Building ${name} (release)"
  cmake -S "$src_dir" -B "$build_release" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$RELEASE_PREFIX" \
    -DCMAKE_C_FLAGS_RELEASE="$RELEASE_C_FLAGS" \
    -DCMAKE_CXX_FLAGS_RELEASE="$RELEASE_CXX_FLAGS" \
    "${extra_args[@]}"
  cmake --build "$build_release" --parallel "$(nproc)"
  cmake --install "$build_release"

  log "Building ${name} (debug)"
  cmake -S "$src_dir" -B "$build_debug" -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_INSTALL_PREFIX="$DEBUG_PREFIX" \
    -DCMAKE_C_FLAGS_DEBUG="$DEBUG_C_FLAGS" \
    -DCMAKE_CXX_FLAGS_DEBUG="$DEBUG_CXX_FLAGS" \
    "${extra_args[@]}"
  cmake --build "$build_debug" --parallel "$(nproc)"
  cmake --install "$build_debug"
}

build_dual_prefix_dep() {
  local name="$1"
  local ref="$2"
  local url="$3"
  shift 3
  local extra_args=("$@")

  clone_or_update "$name" "$url" "$ref"

  local src_dir="${SRC_ROOT}/${name}"
  local build_release="${src_dir}/build_release"
  local build_debug="${src_dir}/build_debug"

  if [[ "$FORCE_REBUILD" == "1" ]]; then
    rm -rf "$build_release" "$build_debug"
  fi

  log "Building ${name} (release)"
  source_ros_setup
  CMAKE_PREFIX_PATH="${RELEASE_PREFIX}:/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH:-}" \
  AMENT_PREFIX_PATH="/opt/ros/${ROS_DISTRO}:${AMENT_PREFIX_PATH:-}" \
  cmake -S "$src_dir" -B "$build_release" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$RELEASE_PREFIX" \
    -DCMAKE_PREFIX_PATH="${RELEASE_PREFIX};/opt/ros/${ROS_DISTRO}" \
    -DCMAKE_C_FLAGS_RELEASE="$RELEASE_C_FLAGS" \
    -DCMAKE_CXX_FLAGS_RELEASE="$RELEASE_CXX_FLAGS" \
    "${extra_args[@]}"
  cmake --build "$build_release" --parallel "$(nproc)"
  cmake --install "$build_release"

  log "Building ${name} (debug)"
  source_ros_setup
  CMAKE_PREFIX_PATH="${DEBUG_PREFIX}:/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH:-}" \
  AMENT_PREFIX_PATH="/opt/ros/${ROS_DISTRO}:${AMENT_PREFIX_PATH:-}" \
  cmake -S "$src_dir" -B "$build_debug" -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_INSTALL_PREFIX="$DEBUG_PREFIX" \
    -DCMAKE_PREFIX_PATH="${DEBUG_PREFIX};/opt/ros/${ROS_DISTRO}" \
    -DCMAKE_C_FLAGS_DEBUG="$DEBUG_C_FLAGS" \
    -DCMAKE_CXX_FLAGS_DEBUG="$DEBUG_CXX_FLAGS" \
    "${extra_args[@]}"
  cmake --build "$build_debug" --parallel "$(nproc)"
  cmake --install "$build_debug"
}

build_ompl_variant() {
  local name="$1"
  local url="$2"
  local ref="$3"
  local install_release="$4"
  local install_debug="$5"

  clone_or_update "$name" "$url" "$ref"

  local src_dir="${SRC_ROOT}/${name}"
  local build_release="${src_dir}/build_release"
  local build_debug="${src_dir}/build_debug"

  if [[ "$FORCE_REBUILD" == "1" ]]; then
    rm -rf "$build_release" "$build_debug"
  fi

  log "Building ${name} (release)"
  source_ros_setup
  CMAKE_PREFIX_PATH="${RELEASE_PREFIX}:/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH:-}" \
  AMENT_PREFIX_PATH="/opt/ros/${ROS_DISTRO}:${AMENT_PREFIX_PATH:-}" \
  cmake -S "$src_dir" -B "$build_release" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$install_release" \
    -DCMAKE_PREFIX_PATH="${RELEASE_PREFIX};/opt/ros/${ROS_DISTRO}" \
    -DCMAKE_C_FLAGS_RELEASE="$RELEASE_C_FLAGS" \
    -DCMAKE_CXX_FLAGS_RELEASE="$RELEASE_CXX_FLAGS" \
    -DOMPL_BUILD_DEMOS=OFF \
    -DOMPL_BUILD_PYBINDINGS=OFF \
    -DBUILD_TESTING=OFF
  cmake --build "$build_release" --parallel "$(nproc)"
  cmake --install "$build_release"

  log "Building ${name} (debug)"
  source_ros_setup
  CMAKE_PREFIX_PATH="${DEBUG_PREFIX}:/opt/ros/${ROS_DISTRO}:${CMAKE_PREFIX_PATH:-}" \
  AMENT_PREFIX_PATH="/opt/ros/${ROS_DISTRO}:${AMENT_PREFIX_PATH:-}" \
  cmake -S "$src_dir" -B "$build_debug" -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_INSTALL_PREFIX="$install_debug" \
    -DCMAKE_PREFIX_PATH="${DEBUG_PREFIX};/opt/ros/${ROS_DISTRO}" \
    -DCMAKE_C_FLAGS_DEBUG="$DEBUG_C_FLAGS" \
    -DCMAKE_CXX_FLAGS_DEBUG="$DEBUG_CXX_FLAGS" \
    -DOMPL_BUILD_DEMOS=OFF \
    -DOMPL_BUILD_PYBINDINGS=OFF \
    -DBUILD_TESTING=OFF
  cmake --build "$build_debug" --parallel "$(nproc)"
  cmake --install "$build_debug"
}

install_source_deps() {
  log "Building RapidJSON from pinned commit"
  build_header_only_dep \
    rapidjson "$RAPIDJSON_COMMIT" https://github.com/Tencent/rapidjson.git \
    -DRAPIDJSON_BUILD_DOC=OFF \
    -DRAPIDJSON_BUILD_EXAMPLES=OFF \
    -DRAPIDJSON_BUILD_TESTS=OFF
  write_rapidjson_fallback_config "$RELEASE_PREFIX"
  write_rapidjson_fallback_config "$DEBUG_PREFIX"

  log "Building nanoflann from pinned commit"
  build_header_only_dep \
    nanoflann "$NANOFLANN_COMMIT" https://github.com/jlblancoc/nanoflann.git \
    -DNANOFLANN_BUILD_EXAMPLES=OFF \
    -DNANOFLANN_BUILD_TESTS=OFF
  write_nanoflann_fallback_config "$RELEASE_PREFIX"
  write_nanoflann_fallback_config "$DEBUG_PREFIX"

  log "Building coal from pinned commit"
  build_dual_prefix_dep \
    coal "$COAL_COMMIT" https://github.com/coal-library/coal.git \
    -DHPP_FCL_HAS_QHULL=ON \
    -DCOAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL=ON \
    -DBUILD_TESTING=OFF \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPython_EXECUTABLE=/usr/bin/python3 \
    -DPython3_EXECUTABLE=/usr/bin/python3

  ln -sf "${RELEASE_PREFIX}/lib/libcoal.so" "${RELEASE_PREFIX}/lib/libhpp-fcl.so"
  ln -sf "${DEBUG_PREFIX}/lib/libcoal.so" "${DEBUG_PREFIX}/lib/libhpp-fcl.so"

  log "Building OMPL 2.0"
  build_ompl_variant \
    ompl \
    https://github.com/ompl/ompl.git \
    "$OMPL_COMMIT_2_0" \
    "${RELEASE_PREFIX}" \
    "${DEBUG_PREFIX}"
}

install_ros_compat_shims() {
  log "Installing ROS compatibility symlinks"
  ${SUDO} mkdir -p /usr/local/lib
  ${SUDO} ln -sf "/opt/ros/${ROS_DISTRO}/lib/liburdf.so" /usr/local/lib/liburdf.so
  ${SUDO} ln -sf "/opt/ros/${ROS_DISTRO}/lib/libkdl_parser.so" /usr/local/lib/libkdl_parser.so
  ${SUDO} ldconfig
}

install_env_file() {
  log "Writing /etc/profile.d/sirrt_dependencies.sh"
  ${SUDO} tee /etc/profile.d/sirrt_dependencies.sh >/dev/null <<EOF_ENV
export ROS_DISTRO=${ROS_DISTRO}
export SIRRT_DEPS_ROOT=${DEPS_ROOT}
export SIRRT_DEPS_RELEASE=${RELEASE_PREFIX}
export SIRRT_DEPS_DEBUG=${DEBUG_PREFIX}

sirrt_use_release() {
  export PATH=${RELEASE_PREFIX}/bin:\$PATH
  export CPATH=${RELEASE_PREFIX}/include:\${CPATH:-}
  export CPLUS_INCLUDE_PATH=${RELEASE_PREFIX}/include:/opt/ros/${ROS_DISTRO}/include:/opt/ros/${ROS_DISTRO}/include/urdf:/opt/ros/${ROS_DISTRO}/include/urdfdom_headers:/opt/ros/${ROS_DISTRO}/include/kdl_parser:\${CPLUS_INCLUDE_PATH:-}
  export LIBRARY_PATH=${RELEASE_PREFIX}/lib:/usr/local/lib:/opt/ros/${ROS_DISTRO}/lib:\${LIBRARY_PATH:-}
  export LD_LIBRARY_PATH=${RELEASE_PREFIX}/lib:/usr/local/lib:/opt/ros/${ROS_DISTRO}/lib:\${LD_LIBRARY_PATH:-}
  export CMAKE_PREFIX_PATH=${RELEASE_PREFIX}:/opt/ros/${ROS_DISTRO}:\${CMAKE_PREFIX_PATH:-}
  export PKG_CONFIG_PATH=${RELEASE_PREFIX}/lib/pkgconfig:\${PKG_CONFIG_PATH:-}
}

sirrt_use_debug() {
  export PATH=${DEBUG_PREFIX}/bin:\$PATH
  export CPATH=${DEBUG_PREFIX}/include:\${CPATH:-}
  export CPLUS_INCLUDE_PATH=${DEBUG_PREFIX}/include:/opt/ros/${ROS_DISTRO}/include:/opt/ros/${ROS_DISTRO}/include/urdf:/opt/ros/${ROS_DISTRO}/include/urdfdom_headers:/opt/ros/${ROS_DISTRO}/include/kdl_parser:\${CPLUS_INCLUDE_PATH:-}
  export LIBRARY_PATH=${DEBUG_PREFIX}/lib:/usr/local/lib:/opt/ros/${ROS_DISTRO}/lib:\${LIBRARY_PATH:-}
  export LD_LIBRARY_PATH=${DEBUG_PREFIX}/lib:/usr/local/lib:/opt/ros/${ROS_DISTRO}/lib:\${LD_LIBRARY_PATH:-}
  export CMAKE_PREFIX_PATH=${DEBUG_PREFIX}:/opt/ros/${ROS_DISTRO}:\${CMAKE_PREFIX_PATH:-}
  export PKG_CONFIG_PATH=${DEBUG_PREFIX}/lib/pkgconfig:\${PKG_CONFIG_PATH:-}
}


export AMENT_TRACE_SETUP_FILES=\${AMENT_TRACE_SETUP_FILES:-}
export AMENT_PYTHON_EXECUTABLE=\${AMENT_PYTHON_EXECUTABLE:-/usr/bin/python3}
export COLCON_TRACE=\${COLCON_TRACE:-}
export COLCON_PREFIX_PATH=\${COLCON_PREFIX_PATH:-}
export COLCON_PYTHON_EXECUTABLE=\${COLCON_PYTHON_EXECUTABLE:-/usr/bin/python3}

if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
  set +u
  source /opt/ros/${ROS_DISTRO}/setup.bash
fi

sirrt_use_release
EOF_ENV

  printf '%s\n' "${RELEASE_PREFIX}/lib" | ${SUDO} tee /etc/ld.so.conf.d/sirrt_dependencies_release.conf >/dev/null
  printf '%s\n' "${DEBUG_PREFIX}/lib" | ${SUDO} tee /etc/ld.so.conf.d/sirrt_dependencies_debug.conf >/dev/null
  ${SUDO} ldconfig
}

print_summary() {
  cat <<EOF_SUMMARY

Done.

Installed prefixes:
  release:      ${RELEASE_PREFIX}
  debug:        ${DEBUG_PREFIX}

To load environment in the current shell:
  source /etc/profile.d/sirrt_dependencies.sh

Profiles:
  sirrt_use_release
  sirrt_use_debug

EOF_SUMMARY
}

main() {
  need_cmd bash
  need_cmd grep
  need_cmd sed
  need_cmd awk

  require_ubuntu_24_04
  ensure_locale
  ensure_ros_repo
  install_apt_packages
  ensure_rosdep
  ensure_dirs

  need_cmd git
  need_cmd cmake
  need_cmd python3

  install_source_deps
  install_ros_compat_shims
  install_env_file
  print_summary
}

main "$@"