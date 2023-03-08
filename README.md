# Swarm-SLAM

## Getting started

First, clone this repository:

```bash
git clone git@gitlab.eecs.umich.edu:palinder/swarm-slam.git
cd swarm-slam
```

If you haven't already, install [Docker](https://www.docker.com/). If you're on Windows and want to interact with Docker from a WSL terminal, follow [these steps from Microsoft](https://docs.microsoft.com/en-us/windows/wsl/tutorials/wsl-containers#install-docker-desktop).

Then, create our container:

```bash
chmod +x make_container.sh
bash ./make_container.sh
```

## Running Docker

Now you have two options for actually opening an interactive shell into the container, a graphical way and a command-line way.

### Graphical Option

1. Go into the Containers tab of Docker Desktop.
2. Press the play button in the row of your `rob530` container.
3. Press the three dots in the row of your `rob530` container and select "Open in terminal".

### Command-Line Option

1. Start the container. This is the command you'll run each time to "boot up" the container.

```bash
docker start rob530
```

2. Open a shell into the container:

```bash
docker exec -it rob530 bash
```

3. When you're done and want to shut down the container, run:

```bash
docker stop rob530
```

## GUI Support

Don't worry about this for now.
You'll need to set up X11 forwarding to use any GUIs from within the Docker container.

- macOS
  - Install [XQuartz](https://www.xquartz.org/).
  - Go into the preferences (`XQuartz > Preferences...` in the upper left of the screen).
  - In the "Security" tab of the "X11 Preferences" window, make sure "Allow connections from network clients" is checked.
  - In a Terminal, run `xhost + 127.0.0.1`
    - In the future, you'll have to run this command to launch XQuartz if it's not currently open.
- Windows (based on [these instructions](https://dev.to/darksmile92/run-gui-app-in-linux-docker-container-on-windows-host-4kde))
  - Install [VcXsrv](https://sourceforge.net/projects/vcxsrv/).
  - Open "XLaunch" from the Start menu.
  - Set up a configuration. The defaults should be fine for everything, except make sure that "Disable access control" is checked on the "Extra settings" page.
  - Note: These instructions are untested as of writing, so feedback is welcome.

### Testing X11 Forwarding

In your Docker container, you can install a simple suite of graphical apps:

```bash
sudo apt install x11-apps
```

Then you can launch some sample programs from your shell:

- `xlogo` opens a window with the X11 logo
- `xcalc` opens a scientific calculator utility
- `ico` opens a window with some animated polyhedrons
- `xeyes` opens a window with... well, I'll let you see for yourself

## Testing ROS

When you enter the shell, you should be in the `/ros_ws` directory. Build the ROS workspace by running:

```bash
colcon build
```

Then, run the unit tests with:

```bash
colcon test
```
