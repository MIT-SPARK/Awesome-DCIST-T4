#!/bin/bash
# ./python_setup.sh [--no-roman] [--no-spark]

install_roman=true
install_spark=true

while :; do
    echo $1
    case $1 in
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

if [ -z "${ADT4_ENV+set}" != set ]; then
    echo "Must set ADT4_ENV"
    exit 1
fi

if [ -z "${ADT4_WS+set}" != set ]; then
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
    cd dependencies/clipper/build
    cmake .. && make && make pip-install

    # Pip installs
    cd $ADT4_WS/src/awesome_dcist_t4/roman
    pip install . --no-deps
    pip install -r $ADT4_WS/src/awesome_dcist_t4/install/roman_requirements.txt

    # Download FastSAM weights
    mkdir -p $ADT4_WS/weights
    cd $ADT4_WS/weights
    gdown 'https://drive.google.com/uc?id=1m1sjY4ihXBU1fZXdQ-Xdj-mDltW-2Rqv'

    popd
fi

##################################
# Install Spark
##################################
if [ "$install_spark" = true ]; then
    python3 -m venv $ADT4_ENV/spark_env --system-site-packages
    source $ADT4_ENV/spark_env/bin/activate
    pip install -r $ADT4_WS/src/awesome_dcist_t4/install/spark_requirements.txt
fi
