#!/usr/bin/env bash

GIT_DEPS_REPO=(tiny_math pybind11 imgui spdlog tiny_renderer tysoc)
GIT_DEPS_USER=(wpumacay pybind wpumacay gabime wpumacay wpumacay)
GIT_DEPS_BRANCH=(tysoc-stable master docking v1.x tysoc-stable master)
GIT_DEPS_DEST=(ext/tiny_math ext/pybind11 ext/imgui ext/spdlog ext/tiny_renderer core)

for i in {0..5}
do
    USER=${GIT_DEPS_USER[$i]}
    REPO=${GIT_DEPS_REPO[$i]}
    BRANCH=${GIT_DEPS_BRANCH[$i]}
    URL=https://github.com/${USER}/${REPO}
    if [ ! -d "${GIT_DEPS_DEST[$i]}" ]
    then
        echo "===> Cloning ${USER}/${REPO} @ github - ${BRANCH} branch"
        git clone --branch=${BRANCH} ${URL} ${GIT_DEPS_DEST[$i]}
    else
        echo "===> Repository ${USER}/${REPO} @ github already checked out"
    fi
done