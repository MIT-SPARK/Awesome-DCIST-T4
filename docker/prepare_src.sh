#!/bin/bash
set -e

# Define the specific source directory name
SRC_DIR="adt4_src"

# Clean up previous source checkout if it exists
if [ -d "$SRC_DIR" ]; then
    echo "--- Removing existing '$SRC_DIR' directory... ---"
    rm -rf "$SRC_DIR"
fi

# Step 1: Clone only the main repository first
echo "--- Cloning Awesome-DCIST-T4 into '$SRC_DIR/awesome_dcist_t4'... ---"
git clone git@github.com:MIT-SPARK/Awesome-DCIST-T4.git "$SRC_DIR/awesome_dcist_t4"

# Use pushd to navigate into the repository directory
pushd "$SRC_DIR/awesome_dcist_t4" > /dev/null

# Step 2: Initialize and clone all submodules efficiently and resiliently
echo "--- Cloning all submodules (shallow clone, will continue on failure)... ---"
git submodule update --init --depth 1 --recursive || true

# Use popd to return to the original directory
popd > /dev/null

# Step 3: Import additional packages as before
echo "--- Importing additional packages with vcs... ---"
vcs import "$SRC_DIR" < "$SRC_DIR/awesome_dcist_t4/install/packages.yaml"

echo "--- Source code is ready in the '$SRC_DIR' directory. ---"