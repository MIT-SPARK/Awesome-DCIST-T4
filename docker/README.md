# Awesome DCIST T4 Docker Environment

A complete Docker environment for building and running the [Awesome DCIST T4](https://github.com/MIT-SPARK/Awesome-DCIST-T4) repository.

This setup provides a self-contained, reproducible environment with **ROS 2 Jazzy**, **NVIDIA CUDA**, and **TensorRT** support. It is designed for two primary workflows:
1.  **Deployment:** Building a final, portable image with the application pre-compiled and ready to run.
2.  **Development:** Mounting your local source code into the container for rapid, iterative development and debugging.

## Prerequisites

Before you begin, ensure you have the following installed on your host machine:

1.  **Git:** [Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git).
2.  **Docker & Docker Compose:** [Install Docker Engine](https://docs.docker.com/engine/install/) (which includes Compose).
3.  **NVIDIA GPU Driver:** The latest proprietary NVIDIA drivers.
4.  **NVIDIA Container Toolkit:** Required for GPU access inside Docker. [Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
5.  **SSH Key for GitHub:** You must have an SSH key configured with your GitHub account to clone the private submodules. [GitHub Docs](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).

## Initial Setup (Required for Both Modes)

Follow these steps once to prepare your environment.

**1. Clone this Repository**

Clone the repository containing this Docker setup.
```bash
git clone git@github.com:MIT-SPARK/Awesome-DCIST-T4.git
cd Awesome-DCIST-T4
```

**2. Configure Your Environment**

Copy the example environment file to `.env`. This file stores your secrets and configuration. **You must edit this file.**
```bash
cp .env_example .env
vim .env # Or your favorite editor
```
Be sure to fill in your API keys and adjust settings like `DOCKER_MAKE_JOBS` if needed. This controlls the number of threads during the catkin build. If you have less than 32GB of RAM make sure to lower this from 8.

**3. Prepare the Source Code**

Run the provided script to clone the `Awesome-DCIST-T4` repository and all its dependencies into a local `adt4_src` directory. This script uses **your host machine's SSH keys**, which is necessary for private submodules.
```bash
chmod +x prepare_source.sh
./prepare_source.sh
```

**4. Prepare Data Directories**

Create the directories on your host that will be used for persistent data storage.
```bash
mkdir data
mkdir adt4_output
```
-   `./data`: Place input files like bag files here.
-   `./adt4_output`: Application output like logs and maps will appear here.

You are now ready to choose a usage mode.

---

## Usage Modes

### 1. Deployment Mode (Recommended for Running)

This mode builds a self-contained image with the entire workspace pre-compiled. The final image is portable and can be run on any machine with Docker and NVIDIA drivers.

**Step 1: Build the Image**

This command builds the final `deployment` image. The `--no-cache` flag is recommended for the first build to ensure a clean state.
```bash
docker compose build --no-cache deploy
```

**Step 2: Start the Container**

Start the service in the background.
```bash
docker compose up -d deploy
```

**Step 3: Access the Container**

You will be dropped into a `zsh` shell inside the container. The ROS 2 workspace is already built and sourced, and the environment is ready to use immediately.
```bash
docker compose exec deploy zsh
```

### 2. Development Mode (Recommended for Coding)

This mode mounts your local `adt4_src` directory into the container. Any code changes you make on your host machine are instantly reflected inside the container, allowing you to use your favorite IDE.

**Step 1: Start the Container**

This command builds the `base` image (with all dependencies) and starts the container.
```bash
docker compose up --build -d dev
```

**Step 2: Access the Container**

Get an interactive shell inside the running container.
```bash
docker compose exec dev zsh
```

**Step 3: Build the Workspace (First Time Only)**

> **Note:** In development mode, you must run the first build manually inside the container.

Run the `colcon build` command. It will use the `DOCKER_MAKE_JOBS` variable from your `.env` file.
```bash
# Inside the dev container
cd /dcist_ws
colcon build --continue-on-error --executor sequential
```
After the build completes, your development environment is ready. You only need to re-run `colcon build` after making C++ changes. Python changes are reflected live.

---

## Key Features & Workflow

### Data Persistence

This setup uses two persistent directories to sync data between your host and the container.

| Host Path        | Container Path         | Purpose                                          |
| ---------------- | ---------------------- | ------------------------------------------------ |
| `./data`         | `/data`                | **Input:** Place ROS bags and other data here.   |
| `./adt4_output`  | `/root/adt4_output`    | **Output:** Logs, maps, and other data appear here. |

**Example:**
1.  Place a bag file in `./data/my_run.bag` on your host.
2.  Inside the container, you can access it at `/data/my_run.bag`.

### GPU & GUI Applications (RVIZ2)

The container is configured to forward GUI applications to your host's display.

**On your host machine**, run this command once to grant permission:
```bash
xhost +local:docker
```

You can now run graphical applications like RVIZ2 from inside the container:
```bash
# Inside the container
rviz2
```

### Build Configuration

You can control the RAM usage of the `colcon build` process by editing `DOCKER_MAKE_JOBS` in your `.env` file.

-   `DOCKER_MAKE_JOBS=8`: (Default) Good for machines with 32GB+ RAM.
-   `DOCKER_MAKE_JOBS=2`: Recommended for laptops with 16GB RAM.
-   `DOCKER_MAKE_JOBS=1`: Uses the least RAM but is the slowest.

You must rebuild the image (`docker compose build --no-cache deploy`) for this change to take effect.

## Common Commands

**Stop and Remove Containers:**
```bash
docker compose down
```

**View Container Logs:**
```bash
docker compose logs -f deploy
```

**Force a Clean Rebuild:**
```bash
docker compose build --no-cache deploy
```

## Included Tools

This container comes with several tools to improve the development experience:
-   **Zsh** as the default shell.
-   **Oh-My-Zsh** for themes and plugins.
-   **Vim** and **Tmux** for terminal-based editing and session management.
-   **Rich** for beautiful Python tracebacks.