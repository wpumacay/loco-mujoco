
# Project Updates

## 12, nov, 2018

![terrain-progress-1](../_imgs/gif_terrain_progress_1.gif)

* Implemented base functionality to interact with mujoco objects. (ext/mjcint)

    1. Change body position
    2. Set actuator ctrls

* Implemented mjcf object handler (ext/mjcf)

    1. Allows to parse mjcf into basic object model
    2. Allows to create models using this basic object model

## 13, nov, 2018

![terrain-progress-2](../_imgs/gif_terrain_progress_2.gif)

* Implemented base abstract terrain generation. (ext/tysocCore)

    1. implemented basic structure of the abstract terrain generation library
    2. implemented sections-type terraing generation

* Refactored repo and updated design

![design-v0](../_imgs/img_tysoc_design_v0.jpg)

## 21, nov, 2018

![terrain-progress-3](../_imgs/gif_terrain_progress_3.gif)

![terrain-progress-4](../_imgs/gif_terrain_progress_4.gif)

![terrain-progress-5](../_imgs/gif_terrain_progress_5.gif)

![terrain-progress-6](../_imgs/gif_terrain_progress_6.gif)

* Implemented framework :

    1. Implemented the full base framework and added wrappers for mujoco
    2. Added perlin noise generator
    3. Added walker and humanoid template

## 23, nov, 2018

* Integrated custom rendering engine into a decoupled visualizer : the visualizer is working and is decoupled of the concrete physics engine used. It updates its data from the underlying data being used by the abstract API, whose contents are written by the concrete implementations.

![terrain-progress-7](../_imgs/gif_terrain_progress_7.gif)

![terrain-progress-8](../_imgs/gif_terrain_progress_8.gif)

## 24, nov, 2018

* Added sensors (agent intrinsic and terrain extrinsic) to the library

![terrain-progress-9](../_imgs/gif_terrain_progress_9.gif)

![terrain-progress-10](../_imgs/gif_terrain_progress_10.gif)

## 25, nov, 2018

* Added terrain generators to replicate the environments from [here](https://www.youtube.com/watch?v=hx_bgoTF7bs)

![terrain-progress-11](../_imgs/gif_terrain_progress_11.gif)

![terrain-progress-12](../_imgs/gif_terrain_progress_12.gif)

![terrain-progress-13](../_imgs/gif_terrain_progress_13.gif)

![terrain-progress-14](../_imgs/gif_terrain_progress_14.gif)

![terrain-progress-15](../_imgs/gif_terrain_progress_15.gif)

## 13, dic, 2018

* Made various changes after poster presentation :

    1. Implemented Kinematic tree agents
    2. Modified agent API
    3. Added small UI
    4. Fixed sensors for new agent API
    5. Refactored a bit

![tysoc-mjc-progress-1](../_imgs/gif_tysocMjc_progress_1_1.gif)

![tysoc-mjc-progress-2](../_imgs/gif_tysocMjc_progress_1_2.gif)

![tysoc-mjc-progress-3](../_imgs/gif_tysocMjc_progress_1_3.gif)

## dic, 2018

* Added starting support for urdf
* Refactored code: parsers
* Added joint sensors
* Added support for mujoco visualizer
* ... (wil check commits later)

## jan, 2019

* Added support for urdf (still some fixes needed)
* Refactored code: design of the core agent functionality
* ... (wil check commits later)

## feb, 2019

* Added support for the rlsim format (from [here](https://github.com/UBCMOCCA/TerrainRLSim/tree/master/data/characters))
* Added a small visualizer/editor? (will add edition functionality after some extra changes)
* ... (will check commits later)

## 17, feb, 2019

* Added dynamic loading functionality: We can load any simulation and visualizer at runtime,
    which will be used later when choosing among various physics backends. See test_runtime.cpp 
    in the examples folder. In the example we only change a line to indicate which visualizer, and
    voila, you just swapped the visualizer, no need to recompile. I will be fixing the other examples
    tomorrow (I kind of leave the old examples unusable).

![tysoc-mjc-progress-dlloading-custom-viz](../_imgs/gif_tysocMjc_progress_dlloading_custom_viz.gif)

![tysoc-mjc-progress-dlloading-mujoco-viz](../_imgs/gif_tysocMjc_progress_dlloading_mujoco_viz.gif)

* Will be working on the python bindings now, so in the next days we will have a usable
  python simulator. It will be better when we start adding the other backends. First Bullet,
  and then PhysX.