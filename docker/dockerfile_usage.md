# AMR2 Dockerfile (X86) Usage Guide 🤖

This guide provides instructions on how to set up and run the **AMR2 ROS2 environment** using **Docker** on an **x86 machine**.


## 1\. Install Docker

First, you need to install [Docker](https://docs.docker.com/engine/install/ubuntu/) on your **Ubuntu system**. Follow the official installation guide.



## 2\. Create Configuration Files

Navigate to the `docker` directory or copy the `Dockerfile` and `requirements.txt` files from the `docker` folder to your preferred directory.



## 3\. Build the Docker Image

Navigate to the directory containing your files and run the build command. Replace `my-ros2-image` with your desired image name.

For `x86` arch:
```zsh
cd ~/shared_files/ros2_ws/docker
docker build -t my-ros2-image .
````

For `arm` arch such as macos silicon:
```zsh
cd ~/shared_files/ros2_ws/docker
docker build -f Dockerfile_arm64 -t my-ros2-image .
````



## 4\. Run the Container for Testing

You can run the container to test it. There are different commands depending on whether you have a GPU.

#### With GPU Support

This command maps your GPU, display, and shared files into the container.

```zsh
docker run -it --rm -e ROS_DOMAIN_ID=2 \
 -v /var/run/docker.sock:/var/run/docker.sock \
 -v /tmp/.x11-unix:/tmp/.x11-unix \
 -v ~/shared_files:/root/shared_files \
 -v /dev:/dev --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
 -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
 --privileged --name "container_name" --hostname "host_name" \
 --net=host my-ros2-image zsh
```

#### Without GPU Support

This command is for systems without a dedicated NVIDIA GPU.

```zsh
docker run -it --rm -e ROS_DOMAIN_ID=2 \
 -v /var/run/docker.sock:/var/run/docker.sock \
 -v /tmp/.x11-unix:/tmp/.x11-unix \
 -v ~/shared_files:/root/shared_files \
 -v /dev:/dev \
 -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
 --privileged --name "container_name" --hostname "host_name" \
 --net=host my-ros2-image zsh
```

#### Windows WSL

```zsh
# If WSL is not installed:
wsl --install
```
Before running, install [Windows Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/) and [VcXsrv](https://sourceforge.net/projects/vcxsrv/) (for visualization). \
In Docker Desktop's `Settings -> Resources -> WSL Integration`, Ensure integration is enabled with additional distros. \
Then, run `Docker Desktop`, `VcXsrvin` and `the following command` in the WSL terminal:
```zsh
docker run -it --rm --name "container_name" \
  --privileged \
  -e DISPLAY="host.docker.internal:0.0" \
  -v ~/shared_files:/root/shared_files \
  my-ros2-image zsh
```

#### MacOS (Testing, no visulization)
Before running, install [MacOS Docker](https://docs.docker.com/desktop/setup/install/mac-install/) and [VcXsrv](https://www.xquartz.org/) (for visualization). 

Then, run `Docker Desktop`, `VcXsrvin` and `the following command` in the terminal:
```zsh
docker run -it --rm \
 -e ROS_DOMAIN_ID=2 \
 -e DISPLAY=host.docker.internal:0 \
 -v /var/run/docker.sock:/var/run/docker.sock \
 -v ~/shared_files:/root/shared_files \
 --privileged \
 --name "container_name" \
 --hostname "host_name" \
 my-ros2-image zsh
```



