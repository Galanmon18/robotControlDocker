# robotControlDocker
Repository containing a Docker container with all the installations and code developed as part of Álvaro Galán Cuenca’s thesis on medical robotics at the University of Málaga

---

## What's Inside

### Docker Image (`Dockerfile`)

Built on top of `ros:noetic-ros-base` (Ubuntu 20.04 + ROS Noetic), the image installs and compiles:

**System tools**
- `git`, `python3-rosdep`, `python3-catkin-tools`, `iputils-ping`

**ROS packages (apt)**
- `ros-noetic-pcl-ros` and `ros-noetic-pcl-conversions` — Point Cloud Library integration with ROS
- `ros-noetic-eigen-conversions` — conversion utilities between Eigen and ROS geometry types

**ROS packages (compiled from source)**
- [`Universal_Robots_ROS_Driver`](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) — official UR driver for ROS Noetic
- [`fmauch/universal_robot`](https://github.com/fmauch/universal_robot/tree/calibration_devel) (`calibration_devel` branch) — robot description and calibration support
- `uma_ur_launch` — lab-specific launch files and robot calibration YAML files for our UR3e setup

All packages are compiled with `catkin_make` at build time. The ROS environment is automatically sourced in every shell session via `.bashrc` and a custom `entrypoint.sh`.

### Docker Compose (`docker-compose.yml`)

Configures the container at runtime:

- `network_mode: host` — the container shares the host network interface directly, so ROS communication and robot connectivity (Ethernet) work without any extra configuration
- `privileged: true` — allows access to USB/serial devices (e.g. haptic interface)
- **Volume mount**: `./workspace` on your machine is mounted as `/catkin_ws/user_ws/src` inside the container. This is where your development packages live — edit locally, compile inside the container
- `ROS_MASTER_URI` and `ROS_IP` set to `localhost` by default

---

## Requirements

- [Docker](https://docs.docker.com/engine/install/) installed on the host machine
- The `ur-ros-driver.tar` image file (provided separately) **or** the ability to build from the `Dockerfile`

---

## Setup

### 1. Install Docker (if not already installed)

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
```

Log out and back in for the group change to take effect.

### 2. The repository

The folder structure should look like this:

```
robot-docker/
├── Dockerfile
├── docker-compose.yml
├── entrypoint.sh
├── uma_ur_launch/
└── workspace/
    └── uma_fp_control/     ← your development packages go here
```

### 3. Load the Docker image

If you need to build from scratch (requires internet, takes 30–60 min):

```bash
docker build -t ur-ros-driver .
```

### 4. (Optional) Add a shell alias

```bash
echo 'alias ros-term="docker compose -f ~/path/to/robot-docker/docker-compose.yml exec ros-robot bash"' >> ~/.bashrc
source ~/.bashrc
```

---

## Daily Usage

### Start the container

Always run this from inside the this folder so the relative volume path `./workspace` resolves correctly:

```bash
cd <your folder name>
docker compose up -d
```

### Open a terminal inside the container

```bash
ros-term
# or without the alias:
docker compose exec ros-robot bash
```

ROS Noetic and the UR driver workspace are sourced automatically. Open as many terminals as needed — each `ros-term` call opens a new shell in the same running container.

### Compile your workspace (required after every container restart)

```bash
cd /catkin_ws/user_ws
catkin_make
```

The compiled binaries live inside the container, not in the volume, so they are lost when the container stops. The source code in `workspace/` is always preserved on your machine.

### Develop

Edit your packages in `<your folder name>/workspace/` using any editor on your host machine (e.g. VSCode). Changes are reflected inside the container immediately via the volume mount. After editing, recompile with `catkin_make`.


### Stop the container

```bash
docker compose down
```

---

## Adding a New Package to the Workspace

Copy your package into `robot-docker/workspace/`:

```bash
cp -r ~/catkin_ws/src/my_package <your folder name>/workspace/
```

Then compile inside the container:

```bash
ros-term
cd /catkin_ws/user_ws && catkin_make
```

No image rebuild needed.

---

## Robot Control Package

The main robot control software is located in:

```text
workspace/uma_fp_control/
```

This package contains all the controllers, force control algorithms, launch files, and utilities developed during the thesis. It also includes its own dedicated `README.md` with detailed documentation describing the available executables, controller architectures, launch files, configuration, and package structure.

Refer to:

```text
workspace/uma_fp_control/README.md
```

for detailed information about the robot control framework.
