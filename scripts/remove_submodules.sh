#!/bin/sh

# --------------------------------------------------------------------- #
# script/no_submodules: Removes any submodules from this repo           #
# --------------------------------------------------------------------- #

for submod in ext/cat1 ext/imgui ext/pybind11 tysoc
do
    echo "Deleting: ${submod}"
    mv "${submod}" "${submod}_tmp"
    git submodule deinit -f -- "${submod}"
    rm -rf ".git/modules/${submod}"
    git rm -f "${submod}"
done