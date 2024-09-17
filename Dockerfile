# NOTE! 22.04 (jammy) have an issue with: vtkXRenderWindowInteractor 
# More info: https://github.com/PointCloudLibrary/pcl/issues/5237 
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PCL_VERSION=pcl-1.13.1
ENV INSTALL_DIR=/opt/pcl

# Update and install required packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    cmake \
    sudo \
    curl \
    ca-certificates \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev \
    libvtk7-dev \
    libqhull-dev \
    libusb-1.0-0-dev \
    libpng-dev \
    libpcap-dev \
    libopenni2-dev \
    pkg-config \
    clang-format

# Clean up to keep the image size small
RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

# Create a directory for building PCL
RUN mkdir -p /usr/src/pcl
WORKDIR /usr/src/pcl

# Clone the PCL repository
RUN git clone --depth 1 --branch $PCL_VERSION https://github.com/PointCloudLibrary/pcl.git .

# Create build directory
RUN mkdir build
WORKDIR /usr/src/pcl/build

# Configure the build and specify the custom installation directory
RUN cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR

# Build and install PCL
# $(nproc) - Uses all processors available
RUN make -j8
RUN make install

# Update the library path
# ${LD_LIBRARY_PATH:-} ensures that if LD_LIBRARY_PATH is undefined, it defaults to an empty string.
ENV LD_LIBRARY_PATH=$INSTALL_DIR/lib:${LD_LIBRARY_PATH-}
ENV PKG_CONFIG_PATH=$INSTALL_DIR/lib/pkgconfig:${PKG_CONFIG_PATH:-}

# Create a new user 'user' with sudo privileges
RUN useradd -m user && \
    echo "user ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/user && \
    chmod 0440 /etc/sudoers.d/user

# Use sed to uncomment the force_color_prompt line in ~/.bashrc
RUN sed -i 's/#force_color_prompt=yes/force_color_prompt=yes/g' /home/user/.bashrc

# Switch to the 'user' you created
USER user

# Default command
CMD ["/bin/bash"]
