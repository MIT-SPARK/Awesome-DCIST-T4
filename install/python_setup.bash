#!/bin/bash
# ./python_setup.sh [--no-roman] [--no-spark] [--no-console]

ci_running=false
install_roman=true
install_spark=true
install_console=true

while :; do
    echo $1
    case $1 in
        --is-ci)
            ci_running=true
            shift
            ;;
        --no-roman)
            install_roman=false
            shift
            ;;
        --no-spark)
            install_spark=false
            shift
            ;;
        --no-console)
            install_console=false
            shift
            ;;
        -h|-?|--help)
            echo "Usage: ./python_setup.sh [--no-roman] [--no-spark] [--no-console]"
            exit
            ;;
        *)
        break
    esac
done

if [[ ! -v ADT4_ENV ]]; then
    echo "Must set ADT4_ENV"
    exit 1
fi

if [[ ! -v ADT4_WS ]]; then
    echo "Must set ADT4_WS"
    exit 1
fi

mkdir -p $ADT4_ENV

##################################
# Install ROMAN
##################################
if [ "$install_roman" = true ]; then
    if [ -d $ADT4_ENV/roman ]; then
        echo "ROMAN environment already exists, skipping creation."
    else
        echo "Creating ROMAN environment."
        python3 -m venv $ADT4_ENV/roman
    fi
    source $ADT4_ENV/roman/bin/activate
    pushd $ADT4_WS/src/awesome_dcist_t4/roman

    # Build CLIPPER
    git submodule update --init --recursive
    mkdir -p dependencies/clipper/build
    pushd dependencies/clipper/build
    cmake .. && make && make pip-install
    popd

    # Pip installs
    pip install -e . --no-deps
    pip install -r $ADT4_WS/src/awesome_dcist_t4/install/roman_requirements.txt
    # FastSAM needs no-build-isolation to prevent setuptools errors
    pip install --no-build-isolation git+https://github.com/CASIA-IVA-Lab/FastSAM.git@4d153e9

    # Download FastSAM weights
    mkdir -p $ADT4_WS/weights
    pushd $ADT4_WS/weights
    gdown 'https://drive.google.com/uc?id=1m1sjY4ihXBU1fZXdQ-Xdj-mDltW-2Rqv'
    # YOLOv8 detector ROMAN uses (via ultralytics) to filter out e.g. people
    MODEL_FILE="yolov8m.pt"
    if [ ! -f "$MODEL_FILE" ]; then
	wget https://github.com/ultralytics/assets/releases/download/v8.3.0/$MODEL_FILE
    else
        echo "$MODEL_FILE already exists, skipping download."
    fi
    popd

    # clean up environment after install
    deactivate
    popd
fi

##################################
# Install Spark
##################################

if [ "$install_spark" = true ]; then
    # make new environment
    python3 -m venv $ADT4_ENV/spark_env --system-site-packages
    source $ADT4_ENV/spark_env/bin/activate

    # move to the ADT4 repo and make sure the state is sane
    pushd $ADT4_WS/src/awesome_dcist_t4
    git submodule update --init

    # install packages and spark_dsg
    if [ "$ci_running" = true ]; then
        pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
    fi
    pip install -r install/spark_requirements.txt
    # fleetlink's agent and bridge re-execute in this environment (FLEETLINK_PYTHON, else
    # $ADT4_ENV/spark_env) and need the zenoh of rmw_zenoh (1.8). Fail loudly: a robot
    # without it has no link to the base in fleetlink mode ("No module named 'zenoh'").
    if ! python -c "import importlib.metadata as m, sys; sys.exit(m.version('eclipse-zenoh') != '1.8.0')"; then
        echo "eclipse-zenoh 1.8.0 is not installed in $ADT4_ENV/spark_env (see install/spark_requirements.txt)"
        exit 1
    fi
    pip install -e ./semantic_inference/semantic_inference
    # hydra_python (hydra/hydra/python) has no pip package; its pure-python image
    # store, which the console reads images with, needs only its source on the
    # path (the compiled bindings stay optional)
    if [ -d hydra/hydra/python/src/hydra_python/image_store ]; then
        echo "$ADT4_WS/src/awesome_dcist_t4/hydra/hydra/python/src" > \
            "$(python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')/hydra_python_src.pth"
    fi

    # install fast-downward
    if [ ! -d $ADT4_WS/src/fast_downward ]; then
        git clone https://github.com/aibasel/downward.git $ADT4_WS/src/fast_downward
        touch $ADT4_WS/src/fast_downward/COLCON_IGNORE
        $ADT4_WS/src/fast_downward/build.py
        echo "*********************************************************************************************************"
        echo "** Please run the following to install fast-downward:                                                  **"
        echo "**                                                                                                     **"
        echo "** sudo ln -s $ADT4_WS/src/fast_downward/fast-downward.py /usr/local/bin/fast-downward                 **"
        echo "**                                                                                                     **"
        echo "*********************************************************************************************************"
    fi

    # download weights
    if [[ "$ci_running" = false ]]; then
        $ADT4_ENV/spark_env/bin/python $ADT4_WS/src/awesome_dcist_t4/install/download_weights.py -d $ADT4_WS/weights \
            yolov8s-world.pt \
            yoloe-26m-seg.pt \
            mobileclip2_b.ts
    fi

    # clean up environment after install
    deactivate
    popd
fi

##################################
# Web console UI (adt4_console)
##################################
# The console's Python dependencies are in spark_requirements.txt. Its browser UI
# is built here (into adt4_console/adt4_console/static, not committed; colcon
# installs it with the package) and needs Node 20.19+ or 22.12+; without it the console
# serves a page saying how to build the UI, and the API.

if [ "$install_console" = true ]; then
    console_frontend=$ADT4_WS/src/awesome_dcist_t4/adt4_console/frontend
    node_ok=false
    if command -v node > /dev/null 2>&1 && command -v npm > /dev/null 2>&1; then
        if node -e 'const [a, b] = process.versions.node.split(".").map(Number); process.exit((a === 20 && b >= 19) || (a === 22 && b >= 12) || a > 22 ? 0 : 1)'; then
            node_ok=true
        fi
    fi
    if [ "$ci_running" = true ]; then
        echo "Skipping the console UI build (--is-ci)."
    elif [ ! -f $console_frontend/package.json ]; then
        echo "Skipping the console UI build: no $console_frontend/package.json."
    elif [ "$node_ok" = false ]; then
        echo "*********************************************************************************************************"
        echo "** Skipping the console UI build: it needs Node 20.19+ or 22.12+ (found: $(node --version 2>/dev/null || echo none)). **"
        echo "** Install Node 22 (nvm or NodeSource; apt's 18.x is too old) and re-run this script, or run:            **"
        echo "**   cd $console_frontend && npm ci && npm run build"
        echo "*********************************************************************************************************"
    else
        echo "Building the console UI in $console_frontend"
        if ! (cd "$console_frontend" && npm ci && npm run build); then
            echo "Console UI build failed; fix the npm error and re-run this script." >&2
            exit 1
        fi
    fi
fi
