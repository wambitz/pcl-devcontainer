# PCL Docker Environment with XServer and VS Code DevContainers

This repository provides a Dockerfile and a DevContainer setup for working with the Point Cloud Library (PCL) in an Ubuntu 20.04 environment. The setup allows you to build your own Docker image and use it with VS Code for development. It also supports XServer for rendering point clouds or visualizations from within Docker.

## Table of Contents
- [Getting Started](#getting-started)
- [Prerequisites](#prerequisites)
- [Setup Instructions](#setup-instructions)
- [Using XServer for Visualization](#using-xserver-for-visualization)
- [Developing with VS Code](#developing-with-vs-code)
- [Known Issues](#known-issues)
- [References](#references)

## Getting Started

This repository allows you to set up a Docker-based development environment for the Point Cloud Library (PCL) using an Ubuntu 20.04 base image. The provided setup supports XServer for graphical output and can be easily integrated with VS Code through its DevContainers feature.

## Prerequisites

Before starting, ensure you have the following installed:

- **Docker**: [Install Docker](https://docs.docker.com/get-docker/)
- **VS Code**: [Download Visual Studio Code](https://code.visualstudio.com/)
  - **Remote Containers Extension**: Install the [Remote - Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- **XServer**: Ensure you have an XServer running for graphical applications.
  - For Linux, XServer is typically available by default.
  - For macOS or Windows, you can install [XQuartz](https://www.xquartz.org/) or [VcXsrv](https://sourceforge.net/projects/vcxsrv/), respectively.

## Setup Instructions

### 1. Clone the Repository

First, clone this repository to your local machine:

```bash
git clone https://github.com/wambitz/pcl-devcontainer
cd pcl-devcontainer
```

### 2. Build the Docker Image

Use the provided Dockerfile to build the image:

```bash
docker build -t pcl-dev .
```

### 3. Using XServer for Visualization

The provided Docker setup supports graphical applications, such as those required for 3D point cloud visualization. To use XServer, ensure you have it installed and running on your host machine.

If you're using macOS or Windows, make sure to configure `XQuartz` or `VcXsrv` correctly and allow connections from the Docker container.

**NOTE**: If you need help with X-Server and Docker you can visit [this page](https://wambitz.github.io/tech-blog/jekyll/update/2024/07/25/gui-applications-with-docker.html) where I explain it depth how to run GUI apps with Docker containers.

### 4. Run the Docker Container

You can do this either from the command line or use the `devcontainer` configuration provided in this repository:

#### 4a. Start the Docker Container with Terminal

Run the container with XServer access enabled:

```bash
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    pcl-dev
```

Ensure XServer is running on your host machine and allows connections from the Docker container. On Linux, you may need to run the following:

```bash
xhost +local:docker
```

#### 4b. Developing with VS Code Devcontainer

If you're using VS Code for development, this repository includes a `devcontainer.json` configuration file. This allows you to work inside the container with all necessary dependencies set up.

Steps:

1. Open this repository in VS Code.
2. Press `F1`, then select **Remote-Containers: Reopen in Container**.
3. VS Code will automatically open the repository inside the Docker container using the settings from `devcontainer.json`.

### 5. Build Projects

```bash
cmake -S . -B build
cmake --build build
```

## Known Issues

- **VTK Rendering Issues**: There is a known issue with VTK on Ubuntu 22.04. To avoid problems with `vtkXRenderWindowInteractor`, we are using Ubuntu 20.04 in this setup. For more information, see this [GitHub issue](https://github.com/PointCloudLibrary/pcl/issues/5237).

## References

- [Point Cloud Library Documentation](https://pointclouds.org/documentation/)
- [Docker Documentation](https://docs.docker.com/)
- [VS Code Remote - Containers](https://code.visualstudio.com/docs/remote/containers)
