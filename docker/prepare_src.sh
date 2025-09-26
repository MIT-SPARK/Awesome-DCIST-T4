#!/bin/bash
set -e

# Define the specific source directory name
SRC_DIR="adt4_src"

# Clean up previous source checkout if it exists
if [ -d "$SRC_DIR" ]; then
    echo "--- Removing existing '$SRC_DIR' directory... ---"
    rm -rf "$SRC_DIR"
fi

echo "--- Cloning Awesome-DCIST-T4 and its submodules into '$SRC_DIR'... ---"
# Use the SSH URL for the main repository
git clone git@github.com:MIT-SPARK/Awesome-DCIST-T4.git "$SRC_DIR/awesome_dcist_t4" --recursive

echo "--- Importing additional packages with vcs... ---"
vcs import "$SRC_DIR" < "$SRC_DIR/awesome_dcist_t4/install/packages.yaml"

echo "--- Source code is ready in the '$SRC_DIR' directory. ---"