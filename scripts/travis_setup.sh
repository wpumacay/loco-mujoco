#!/bin/sh

# --------------------------------------------------------------------- #
# script/setup: Sets up this repo by cloning all required dependencies  #
# --------------------------------------------------------------------- #

echo "======== Cloning dependencies -> ext/ ==========="

# rendering engine for visualizer
echo "==> Cloning wpumacay/cat1 @ github - master branch"
git clone --depth=1 https://github.com/wpumacay/cat1.git ext/cat1

# using own imgui version to add some extra cmake-files
echo "==> Cloning wpumacay/imgui @ github - master branch"
git clone --depth=1 https://github.com/wpumacay/imgui.git ext/imgui

# used for generating pytysoc|python bindings
echo "==> Cloning pybind/pybind11 @ github - master branch"
git clone --depth=1 https://github.com/pybind/pybind11.git ext/pybind11

echo "========== Cloning core library -> core ========="

# core interface of this library (recall this repo is an extension with support for bullet)
echo "==> Cloning wpumacay/tysoc @ github - master branch"
git clone --depth=1 https://github.com/wpumacay/tysoc.git core

