
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

* Implemented base abstract terrain generation. (ext/tysocterrain)

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