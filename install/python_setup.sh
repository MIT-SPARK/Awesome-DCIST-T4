source ~/.zshrc
mkdir -p $DCIST_ENV

##################################
# Install ROMAN
##################################
python3 -m virtualenv $DCIST_ENV/roman --download
source $DCIST_ENV/roman/bin/activate
pushd $DCIST_WS/src/awesome_dcist_t4/roman

# Build CLIPPER
git submodule update --init --recursive
mkdir -p dependencies/clipper/build
cd dependencies/clipper/build
cmake .. && make && make pip-install

# Pip installs
cd $DCIST_WS/src/awesome_dcist_t4/roman
pip install . --no-deps
pip install -r $DCIST_WS/src/awesome_dcist_t4/install/roman_requirements.txt

# Download FastSAM weights
mkdir -p $DCIST_WS/weights
cd $DCIST_WS/weights
gdown 'https://drive.google.com/uc?id=1m1sjY4ihXBU1fZXdQ-Xdj-mDltW-2Rqv'

popd