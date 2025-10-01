#!/bin/bash
#
# Unified build script for the Awesome DCIST T4 Docker Environment.
# Handles source code preparation, building, and running containers.
# Supports both SSH and HTTPS checkout methods.
#
set -e

# --- Script Configuration ---
SRC_DIR="adt4_src"
REPO_URL_SSH="git@github.com:MIT-SPARK/Awesome-DCIST-T4.git"
REPO_URL_HTTPS="https://github.com/MIT-SPARK/Awesome-DCIST-T4.git"

# --- Argument Parsing ---
USE_HTTPS=false
for arg in "$@"; do
    if [ "$arg" == "--https" ]; then
        USE_HTTPS=true
        shift 
    fi
done
MODE=$1

# --- Helper Functions ---
usage() {
    echo "Usage: $0 [--https] {fulldev|full|prepare|dev|deploy|down|clean}"
    echo
    echo "Options:"
    echo "  --https     Use HTTPS for all git operations instead of SSH."
    echo
    echo "Commands:"
    echo "  fulldev   (One-Step) Prepares source and builds/starts the DEVELOPMENT container."
    echo "  full      (One-Step) Prepares source and builds/starts the DEPLOYMENT container."
    echo "  prepare   Clones all source code. Fails if './${SRC_DIR}' already exists."
    echo "  dev       Builds and starts the development container."
    echo "  deploy    Builds and starts the deployment container."
    echo "  down      Stops and removes all running containers for this project."
    echo "  clean     Stops containers and removes the source, output, and data directories (prompts for confirmation)."
    echo
}

do_prepare() {
    echo "--- Preparing Source Code ---"
    
    # --- SAFETY CHECK: Fail if the source directory already exists ---
    if [ -d "$SRC_DIR" ]; then
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        echo "!! ERROR: Source directory './${SRC_DIR}' already exists. !!"
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        echo "To prevent accidental data loss, this script will not automatically delete it."
        echo "Please choose one of the following options:"
        echo "  1. To perform a full cleanup, run: ./build.sh clean"
        echo "  2. To only remove the source, run: rm -rf ${SRC_DIR}"
        echo
        echo "Aborting preparation."
        exit 1
    fi

    if [ "$USE_HTTPS" = true ]; then
        echo "--> Using HTTPS for all git operations."
        REPO_URL=$REPO_URL_HTTPS
    else
        echo "--> Using SSH for all git operations."
        REPO_URL=$REPO_URL_SSH
    fi

    echo "--> Cloning Awesome-DCIST-T4 into '${SRC_DIR}/awesome_dcist_t4'..."
    git clone "$REPO_URL" "${SRC_DIR}/awesome_dcist_t4"

    pushd "${SRC_DIR}/awesome_dcist_t4" > /dev/null
    
    if [ "$USE_HTTPS" = true ]; then
        echo "--> Configuring submodules to use HTTPS..."
        git config --local url."https://github.com/".insteadOf "git@github.com:"
    fi

    echo "--> Cloning submodules individually (will continue on failure)..."
    git submodule sync --recursive
    git submodule foreach --recursive 'git submodule update --init --depth 1 "$path" || echo "--> WARNING: Failed to clone submodule: $path. Continuing..."'
    popd > /dev/null

    echo "--> Importing additional packages with vcs (inside a temporary container)..."
    echo "--> Building the 'dev' image to ensure vcs tool is available..."
    docker compose build dev

    echo "--> Running vcs import with correct host permissions..."
    docker compose run --rm \
      --user "$(id -u):$(id -g)" \
      -v "$(pwd)/${SRC_DIR}":/ws/src \
      dev \
      vcs import src < "src/awesome_dcist_t4/install/packages.yaml"

    echo "--- Source code is ready in './${SRC_DIR}' directory. ---"
}

do_deploy() {
    echo "--- Building and Starting Deployment Environment ---"
    echo "--> Building the 'deploy' image (this will take a while)..."
    docker compose build --no-cache deploy
    echo "--> Starting the 'deploy' container in the background..."
    docker compose up -d deploy
    echo "---"
    echo "Deployment container is running. Access it with:"
    echo "docker compose exec deploy zsh"
    echo "---"
}

do_dev() {
    echo "--- Building and Starting Development Environment ---"
    echo "--> Building the 'dev' image (this may take a while on first run)..."
    docker compose build dev
    echo "--> Starting the 'dev' container in the background..."
    docker compose up -d dev
    echo "---"
    echo "Development container is running. Access it with:"
    echo "docker compose exec dev zsh"
    echo "---"
}

# --- Main Logic ---
if [[ -z "$MODE" ]]; then
    usage
    exit 1
fi

case $MODE in
    fulldev)
        echo "--- Starting Full DEV Build Process (Prepare + Dev) ---"
        do_prepare
        do_dev
        echo "--- Full Dev Build Process Complete! ---"
        ;;

    full)
        echo "--- Starting Full DEPLOY Build Process (Prepare + Deploy) ---"
        do_prepare
        do_deploy
        echo "--- Full Deploy Build Process Complete! ---"
        ;;

    prepare)
        do_prepare
        ;;

    dev)
        do_dev
        ;;

    deploy)
        do_deploy
        ;;

    down)
        echo "--- Stopping and Removing Containers ---"
        docker compose down
        echo "--- Done. ---"
        ;;

    clean)
        echo "--- Cleaning Up Workspace ---"
        echo "--> This will stop and remove all containers."
        docker compose down
        
        echo
        echo "WARNING: This will permanently delete the following directories:"
        echo "  - ./${SRC_DIR}"
        echo "  - ./adt4_output"
        echo "  - ./data"
        echo
        
        read -p "Are you sure you want to continue? [y/N] " CONFIRM
        lower_confirm=${CONFIRM,,}

        if [[ "$lower_confirm" == "y" || "$lower_confirm" == "yes" ]]; then
            echo "--> Proceeding with cleanup..."
            echo "--> Removing source directory: ./${SRC_DIR}"
            rm -rf "${SRC_DIR}"
            echo "--> Removing output directory: ./adt4_output"
            rm -rf "adt4_output"
            echo "--> Removing data directory: ./data"
            rm -rf "data"
            echo "--- Workspace cleaned. ---"
        else
            echo "--- Clean cancelled. ---"
        fi
        ;;

    *)
        usage
        exit 1
        ;;
esac
