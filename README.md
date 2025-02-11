# SMDPS System

## Prerequisites

* A host PC with Docker and Git installed.

## Installation

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/samkwok-hkclr/smdps_ws
```

### 2. Build the Docker images

```bash
cd smdps_ws
docker build --tag ros2_humble_smdps_dev:v0 -f DockerfileRos2Humble --progress=plain --no-cache .
docker build --tag ros2_humble_smdps_sys:v0 -f DockerfileSmdpsSys --progress=plain --no-cache .
```

### 3. Run the Docker Containers

```bash
docker compose up -d
```