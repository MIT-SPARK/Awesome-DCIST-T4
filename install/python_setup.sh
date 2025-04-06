source $DCIST_ENV/bin/activate

# Install ROMAN
pushd $DCIST_WS/src/awesome_dcist_t4/roman

git submodule update --init --recursive
mkdir dependencies/clipper/build
cd dependencies/clipper/build
cmake .. && make && make pip-install

cd $DCIST_WS/src/awesome_dcist_t4/roman
pip install . --no-deps
pip install -r $DCIST_WS/src/awesome_dcist_t4/install/requirements.txt

popd