#!/bin/sh

# --------------------------------------------------------------------- #
# script/setup: Sets up this repo by cloning all required dependencies  #
# --------------------------------------------------------------------- #

for repo in ext/cat1 ext/imgui ext/pybind11 core
do
    echo "Checking status: ${repo}"
    cd "${repo}" 
    git status 
    cd "../.."
    echo "----------------------------------------------------"
done