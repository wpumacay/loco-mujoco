**Status**: Heavy development (you might run into crashes as the library is being developed, sorry in advance).

# tysocMjc: A locomotion toolkit, with MuJoCo as physics backend.

[![Build Status](https://travis-ci.com/wpumacay/tysocMjc.svg?branch=master)](https://travis-ci.com/wpumacay/tysocMjc)

[gif-demo-sample](https://media.giphy.com/media/ZDEAQSUraLao0fOhHi/giphy.gif)

<!--![intro-1](https://media.giphy.com/media/ZcaynbjABz69Zyj9n9/giphy.gif) -->
<!--![intro-2](https://media.giphy.com/media/loMkXQHDRSeHC8s2dy/giphy.gif) -->
<!--![intro-3](https://media.giphy.com/media/9S1zriY4MMt8LjOoSq/giphy.gif)-->

This is an instance of my [**tysoc**](https://github.com/wpumacay/tysocCore) framework for locomotion, 
using [**MuJoCo**](http://mujoco.org) as physics backend. As explained in the core repository, the idea
is to provide a kind of abstraction layer on top of various physics backends, and allow you to just
focus on making your experiment regardless of the details of each specific backend.

I will be adding more documentation as I develop the library, and sorry in advance as I might forget 
to update the docs from time to time. However, one main objective is to write comprehensive documentation, 
and I will be doing it on the go. If you have any suggestions/issues, just post an issue or contact me 
at wpumacay@gmail.com .

## Setup (WIP)

### Requirements

#### Ubuntu >= 16.04

```bash
sudo apt install make cmake pkg-config
sudo apt install libassimp-dev libglfw3-dev libglew-dev
```

#### MuJoCo

Install [MuJoCo](https://www.roboti.us/index.html) in your system (to have MuJoCo as an available 
backend). The build rules expect the MuJoCo libraries to be extracted in `~/.mujoco/mujoco200_linux`, 
and the license key placed in `~/.mujoco/mjkey.txt`.

### Building

#### Ubuntu >= 16.04

```bash
# clone this repository (comes without dependencies)
git clone https://github.com/wpumacay/tysocMjc.git
# clone the third_party dependencies
cd tysocMjc && ./scripts/setup_dependencies.sh
# build the project
mkdir build && cd build && cmake .. && make -j4
# try an example (cwd=build)
./examples/cpp/agents/test_agents mjcf ant
```