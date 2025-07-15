# Smart Medicine Dispensing and Packaging System

## Prerequisites

* A pre-configured CAN module with SocketCAN support.

## Installation

### 1. Clone the Repository

Clone the repository along with its submodules:

```bash
git clone --recurse-submodules https://github.com/HKCLR2021/smdps_wcs
```

### 2. Install the Dependencies

Initialize rosdep, update it, and install the required dependencies:

```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -r -y
```

### 3. Build packages

Compile the workspace in Release mode:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Optional: Docker Images 

If using Docker, build the images with:

```bash
docker compose build
```

