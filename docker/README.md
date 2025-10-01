The script is called:
build_adt4_docker.sh

and this is my current README
Of course. You've provided a great base README. I will merge it with the final, corrected workflows and the additional configuration options we developed.

This final version corrects the development mode instructions, adds the optional TensorRT build flag, and ensures all commands and file names are consistent with the complete setup.

Here is the complete `README.md` file in Markdown format.

---

# Awesome DCIST T4 Docker Environment

A complete Docker environment for building and running the [Awesome DCIST T4](https://github.com/MIT-SPARK/Awesome-DCIST-T4) repository.

This setup provides a self-contained, reproducible environment with **ROS 2 Jazzy**, **NVIDIA CUDA**, and **TensorRT** support. It is designed for two primary workflows:
1.  **Deployment:** Building a final, portable image with the application pre-compiled and ready to run.
2.  **Development:** Mounting your local source code into a pre-built container for rapid, iterative development and debugging.

## Prerequisites

Before you begin, ensure you have the following installed on your host machine:

1.  **Git:** [Install Git](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git).
2.  **Docker & Docker Compose:** [Install Docker Engine](https://docs.docker.com/engine/install/) (which includes Compose).
3.  **NVIDIA GPU Driver:** The latest proprietary NVIDIA drivers.
4.  **NVIDIA Container Toolkit:** Required for GPU access inside Docker. [Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
5.  **SSH Key for GitHub:** You must have an SSH key configured with your GitHub account to clone the private submodules. [GitHub Docs](https://docs.github.com/en/authentication/connecting-to-github-with-ssh).

## Initial Setup (Required for Both Modes)

Follow these steps once to prepare your environment.

**1. Clone the Main Repository**

Clone the `Awesome-DCIST-T4` repository, which contains the `docker` directory.
```bash
git clone git@github.com:MIT-SPARK/Awesome-DCIST-T4.git
cd Awesome-DCIST-T4/docker
```

**2. Configure Your Environment**

Copy the example environment file to `.env`. This file stores your secrets and configuration. **You must edit this file.**
```bash
cp .env_example .env
vim .env # Or your favorite editor
```
Be sure to fill in your API keys and adjust the build configuration settings as needed.

**3. Prepare the Source Code**

Run the provided script to clone all necessary source code into a local `adt4_src` directory. This script uses **your host machine's SSH keys**, which is necessary for private submodules.
```bash
chmod +x prepare_src.sh
./prepare_src.sh
```
> **Note:** This will still work if some repositories aren't accessible to you. However, features in those repositories will not be available. For example, if you don't have access to `hydra-multi`, map fusion won't work.

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

This mode builds a self-contained, clean image with the entire workspace pre-compiled. The final image is portable and can be run on any machine with Docker and NVIDIA drivers.

**Step 1: Build the Image**

This command builds the final `deployment` image. Use `--no-cache` for the first build or after making changes to the `Dockerfile`.
```bash
docker compose build --no-cache deploy
```

**Step 2: Start the Container**

Start the service in the background.
```bash
docker compose up -d deploy
```

**Step 3: Access the Container**

You will be dropped into a `zsh` shell. The ROS 2 workspace is already built and sourced, and the environment is ready to use immediately.
```bash
docker compose exec deploy zsh
```

### 2. Development Mode (Recommended for Coding)

This mode provides a pre-built container and mounts your local `adt4_src` directory inside it. Code changes on your host are instantly reflected in the container.

**Step 1: Build the Development Image**

The first time, you need to build the `dev` image. This will compile the entire workspace inside the container.
```bash
docker compose build --no-cache dev
```

**Step 2: Start the Container**

Start the `dev` service in the background. Subsequent starts will be very fast.
```bash
docker compose up -d dev
```

**Step 3: Access the Container**

Get an interactive shell. The environment is fully built and sourced, with your local code mounted and ready for editing.
```bash
docker compose exec dev zsh
```

**Step 4: Making Code Changes**
-   **Python/Launch/Config changes:** These are live instantly. Just re-run your `ros2` command.
-   **C++ changes:** You only need to recompile the affected packages inside the container.
    ```bash
    # Inside the dev container
    cd /dcist_ws
    colcon build --packages-select <your_changed_package_name>
    ```

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

You can control build options by editing your `.env` file. You must rebuild the image for these changes to take effect.

-   **`DOCKER_MAKE_JOBS`**: Controls the number of parallel jobs during the `colcon build` to manage RAM usage. If you have less than 32GB of RAM, consider lowering this from the default of 8.
-   **`DOCKER_WITH_TENSORRT`**: Set to `true` (default) or `false` to enable/disable the installation of NVIDIA TensorRT, which is required for online semantic inference.

## Common Commands

**Stop and Remove Containers:**
```bash
docker compose down
```

**View Container Logs:**
```bash
docker compose logs -f deploy
```

**Force a Clean Rebuild of a Service:**
```bash
# For the development container
docker compose build --no-cache dev

# For the deployment container
docker compose build --no-cache deploy
```

## Included Tools

This container comes with several tools to improve the development experience:
-   **Zsh** as the default shell.
-   **Oh-My-Zsh** for themes and plugins.
-   **Vim** and **Tmux** for terminal-based editing and session management.
-   **Rich** for beautiful Python tracebacks.


Please update the current README for the new script