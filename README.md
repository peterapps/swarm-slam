# Swarm-SLAM

A full ROS 2 Foxy development environment in a Ubuntu 20.04 virtual machine or Docker container, capable of running [Swarm-SLAM](https://github.com/MISTLab/Swarm-SLAM).

## Getting started

First, clone this repository:

```bash
git clone git@gitlab.eecs.umich.edu:palinder/swarm-slam.git
cd swarm-slam
```

## Option 1: Virtual Machine
We have tested Swarm-SLAM in VMWare Fusion 13. We provide a script `setup_ubuntu_vm.sh` which installs the VMware guest tools, enabling you to mount this repository from your host OS in the guest OS. It also installs all of the dependencies for Swarm-SLAM. You can try just copy and pasting each individual command into a shell, just in case there are any errors.

We also provide a script `rob530_setup.bash` which provides helpful aliases and Bash functions. You can add the line `source rob530_setup.bash` to your `~/.bashrc` to expose them. The script defines constants at the top as follows:

```bash
WS_DIR=~/ros_ws
SHARED_DIR=/mnt/hgfs/swarm-slam
REPO_DIR=${WS_DIR}/src/swarm-slam
DATA_DIR=${REPO_DIR}/data/ros2_data 
DATA_INSTALL_DIR=${WS_DIR}/install/cslam_experiments/share/cslam_experiments/data
```

You can modify these paths accordingly to however your file system is organized. For example, if you share a folder from your host OS called `swarm-slam`, you'll want `SHARED_DIR` to be `/mnt/hgfs/swarm-slam`.

## Option 2: Docker

If you haven't already, install [Docker](https://www.docker.com/). If you're on Windows and want to interact with Docker from a WSL terminal, follow [these steps from Microsoft](https://docs.microsoft.com/en-us/windows/wsl/tutorials/wsl-containers#install-docker-desktop).

Then, create our container:

```bash
make image
make container
```

If there's every a change to the `Dockerfile` that requires you to remake the container, first remove the existing container with:

```bash
make rm
```

### Running Docker

Now you have two options for actually opening an interactive shell into the container, a graphical way and a command-line way.

#### Graphical Option

1. Go into the Containers tab of Docker Desktop.
2. Press the play button in the row of your `rob530` container.
3. Press the three dots in the row of your `rob530` container and select "Open in terminal".

#### Command-Line Option

1. Start the container. This is the command you'll run each time to "boot up" the container.

```bash
make start
```

2. Open a shell into the container:

```bash
make shell
```

3. When you're done and want to shut down the container, run:

```bash
make stop
```

### GUI Support

The Docker container includes a NoVNC server, so you can visit `http://localhost:6080` in your web browser to interact with a desktop environment.

## Testing the repo

When you enter the shell, you should be in the `/ros_ws` directory. Build the ROS workspace by running:

```bash
colcon build
```

Then, run the unit tests with:

```bash
colcon test
```

If you are using the `rob530_setup.bash` script we provided, you can equivalently run:
```bash
buildws
testws
```

## Datasets

The Swarm-SLAM paper refers to eight sequences tested from five different datasets:
| | # Robots | Sensors | Total Length (m) | Size (GB) |
|----------------|----------|----------------|------------------|-----------|
| [KITTI](https://www.cvlibs.net/datasets/kitti/) 00 | 2 | stereo | 3835 | 26.4 |
| [KITTI-360](https://www.cvlibs.net/datasets/kitti-360/) 09 | 5 | lidar | 10714 | 26.9 |
| [GrAco](https://sites.google.com/view/graco-dataset) Ground | 3 | lidar | 1427 | 72.9 |
| [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) Gate | 3 | lidar | 484 | 8.3 |
| [S3E](https://github.com/PengYu-Team/S3E) Square | 3 | stereo & lidar | 2286 | 19.1 |
| S3E College | 3 | stereo & lidar | 3335 | 31.7 |
| S3E Playground | 3 | stereo & lidar | 1232 | 8.7 |
| S3E Laboratory | 3 | stereo & lidar | 468 | 10.3 |

We have tested on KITTI-360, M2DGR, and S3E. In particularly, we provide a ROS package `kitti_utils` with scripts that are useful for converting the raw KITTI 360 dataset into a rosbag with the topics that Swarm-SLAM expects. Another helpful tool is [rosbags-convert](https://ternaris.gitlab.io/rosbags/topics/convert.html), which can convert the ROS 1 bag to a ROS 2 bag. You can then split a bag into "multiple" different robots with `kitti_utils/scripts/split_bag.sh`.

To launch a dataset, use the launch files in `cslam_experiments`. For example, to launch KITTI-360, you can run:
```bash
ros2 launch cslam_experiments kitti360_lidar.launch.py
```
If you are running in a virtual machine, you may have limited bandwidth reading the bag file, which can cause warnings are message starvation. To suppress these, use grep:
```bash
ros2 launch cslam_experiments kitti360_lidar.launch.py | grep -v "read-ahead-queue-size"
```

To run the provided visualization RViz files, you can run:
```bash
ros2 launch cslam_visualization visualization_lidar.launch.py
```