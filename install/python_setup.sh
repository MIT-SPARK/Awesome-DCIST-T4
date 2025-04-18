source ~/.zshrc
mkdir -p $ADT4_ENV

##################################
# Install ROMAN
##################################
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

##################################
# Install Spark
##################################
python3 -m venv $ADT4_ENV/spark_env
source $ADT4_ENV/spark_env/bin/activate
pip install -r $ADT4_WS/src/awesome_dcist_t4/install/spark_requirements.txt
