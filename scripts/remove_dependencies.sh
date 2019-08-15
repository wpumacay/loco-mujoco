#!/bin/sh

# ---------------------------------------------------------------------------------- #
# script/remove_dependencies: Removes the dependencies (be careful of your changes)  #
# ---------------------------------------------------------------------------------- #

echo "======================= Removing dependencies ================================="

echo "==> Removing ext/cat1 dependency (wpumacay/cat1 @ github)"
rm -rf ext/cat1

echo "==> Removing ext/imgui dependency (wpumacay/imgui @ github)"
rm -rf ext/imgui

echo "==> Removing ext/pybind11 dependency (pybind/pybind11 @ github)"
rm -rf ext/pybind11

echo "==> Removing core dependency (wpumacay/tysoc @ github)"
rm -rf core

