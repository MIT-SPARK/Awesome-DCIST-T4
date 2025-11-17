#!/bin/bash
# ./python_setup.sh [--no-roman] [--no-spark]

ci_running=false
install_roman=true
install_spark=true

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
        -h|-?|--help)
            echo "Usage: ./python_setup.sh [--no-roman] [--no-spark]"
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
    python3 -m venv $ADT4_ENV/roman
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

    # Download FastSAM weights
    mkdir -p $ADT4_WS/weights
    pushd $ADT4_WS/weights
    gdown 'https://drive.google.com/uc?id=1m1sjY4ihXBU1fZXdQ-Xdj-mDltW-2Rqv'
    MODEL_FILE="yolov7.pt"
    if [ ! -f "$MODEL_FILE" ]; then
	wget https://github.com/WongKinYiu/yolov7/releases/download/v0.1/$MODEL_FILE
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
    pip install ./spark_dsg "numpy<2"

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
    mkdir -p $ADT4_WS/weights
    pushd $ADT4_WS/weights
    MODEL_FILE="yolov8s-world.pt"
    if [[ ! -f "$MODEL_FILE" && "$ci_running" = false ]]; then
    	wget https://github.com/ultralytics/assets/releases/download/v8.3.0/$MODEL_FILE
    else
    	echo "$MODEL_FILE already exists, skipping download."
    fi
    popd

    # clean up environment after install
    deactivate
    popd
fi
