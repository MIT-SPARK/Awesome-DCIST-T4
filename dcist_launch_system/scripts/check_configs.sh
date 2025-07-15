#!/bin/bash
set -e
cd $ADT4_WS/src/awesome_dcist_t4/dcist_launch_system
if [[ `git status --porcelain` ]]; then
    echo "Diff detected *before* config generation. Should not happen!"
    exit 1
fi

./scripts/generate_configs.sh

if [[ `git status --porcelain` ]]; then
    echo "Diff detected. Committed configs inconsistent with config generators"
    git --no-pager diff
    exit 1
fi
