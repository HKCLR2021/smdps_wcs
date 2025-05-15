# Smart Medicine Dispensing and Packaging System

## Prerequisites

* A host PC with Docker and Git installed.
* CAN Module installed.

## Installation

### 1. Clone the Repository

```bash
git clone --recurse-submodules https://github.com/samkwok-hkclr/smdps_wcs
```

### 2. Build the Containers

```bash
docker compose build
```

### 3. Run the Containers

```bash
docker compose up --detach
```