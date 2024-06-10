# Fixposition SDK docker image for CI (building, unit tests)
# To build say: ./docker.sh build

FROM ros:noetic-ros-base

RUN DEBIAN_FRONTEND=noninteractive apt-get -y update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install \
        build-essential cmake libyaml-cpp-dev libboost-dev libboost-stacktrace-dev zlib1g-dev python3-pip git curl \
        python3-catkin-tools doxygen graphviz libeigen3-dev sqlite3 libsqlite3-dev libtiff-dev libcurl4-openssl-dev

RUN curl https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add - && \
    echo "deb http://apt.llvm.org/focal/ llvm-toolchain-focal-17 main" > /etc/apt/sources.list.d/llvm.list && \
    DEBIAN_FRONTEND=noninteractive apt-get -y update && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install clang-format-17 && \
    DEBIAN_FRONTEND=noninteractive ln -s /usr/bin/clang-format-17 /usr/bin/clang-format && \
    DEBIAN_FRONTEND=noninteractive ln -s /usr/bin/clang-format-diff-17 /usr/bin/clang-format-diff

# We need a more recent gtest than what comes with Ubuntu focal
RUN curl -L https://github.com/google/googletest/archive/refs/tags/v1.13.0.tar.gz -o /tmp/gtest.tar.gz && \
    mkdir /tmp/gtest && \
    cd /tmp/gtest && \
    tar --strip-components=1 -xzvf ../gtest.tar.gz && \
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF && \
    cmake --build build --parallel 4 && \
    cmake --install build && \
    cd / && \
    rm -rf /tmp/gtest.tar.gz /tmp/gtest

# We need a more recent capnp than what comes with Ubuntu focal
RUN curl -L https://capnproto.org/capnproto-c++-1.0.2.tar.gz -o /tmp/capnp.tar.gz && \
    mkdir /tmp/capnp && \
    cd /tmp/capnp && \
    tar --strip-components=1 -xzvf ../capnp.tar.gz && \
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF && \
    cmake --build build --parallel 4 && \
    cmake --install build && \
    cd / && \
    rm -rf /tmp/proj.tar.gz /tmp/proj

# We need a more recent PROJ than what comes with Ubuntu focal
RUN curl -L https://download.osgeo.org/proj/proj-9.4.1.tar.gz -o /tmp/proj.tar.gz && \
    mkdir /tmp/proj && \
    cd /tmp/proj && \
    tar --strip-components=1 -xzvf ../proj.tar.gz && \
    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF && \
    cmake --build build --parallel 4 && \
    cmake --install build && \
    cd / && \
    rm -rf /tmp/proj.tar.gz /tmp/proj

# For devcontainer, see .devcontainer/devcontainer.json
RUN adduser fixposition

# Pre-install commit hooks for root and fixposition
COPY .pre-commit-config.yaml /tmp
RUN python3 -m pip install pre-commit && \
    bash -c 'mkdir /tmp/pc1; cd /tmp/pc1; cp ../.pre-commit-config.yaml .; git init; pre-commit install-hooks' && \
    sudo -u fixposition bash -c 'mkdir /tmp/pc2; cd /tmp/pc2; cp ../.pre-commit-config.yaml .; git init; pre-commit install-hooks' && \
    rm -rf /tmp/.pre-commit-config.yaml /tmp/pc1 /tmp/pc2

